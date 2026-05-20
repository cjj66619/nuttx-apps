/****************************************************************************
 * apps/examples/inferd/inferd_main.c
 *
 * Distributed inference daemon for openvela ESP32-S3 mesh.
 *
 * Modes:
 *   inferd local  [N]          - run N local inferences, report latency
 *   inferd server              - TCP worker: accept tasks, run TFLite, reply
 *   inferd bench  <ip> [N]     - compare local vs offloaded latency
 *
 * Wire protocol (server mode, port INFERD_PORT):
 *   request:  4 bytes  float32  (normalized sensor input, little-endian)
 *   response: 8 bytes  float32 result + uint32 latency_us
 *
 * Build: CONFIG_EXAMPLES_INFERD=y in defconfig
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <math.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/* TFLite Micro C interface — real or mock */
#ifdef CONFIG_EXAMPLES_INFERD_TFLITE
extern int tflm_infer_init(void);
extern int tflm_infer_once(float input, float *output);
#else
static int tflm_infer_init(void) { return 0; }
static int tflm_infer_once(float input, float *output)
{
  /* Mock: fast sinf approximation, ~1us latency */
  *output = sinf(input * 3.14159265f) * 0.5f + 0.5f;
  return 0;
}
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define INFERD_PORT       4446
#define DEFAULT_N         100
#define BENCH_RUNS        50

/****************************************************************************
 * Helpers
 ****************************************************************************/

static uint32_t now_us(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint32_t)(ts.tv_sec * 1000000UL + ts.tv_nsec / 1000UL);
}

/* Deterministic synthetic sensor reading in [-1, 1] */
static float synth_input(int i)
{
  return sinf((float)i * 0.1f) * 0.8f + 0.1f * ((float)(i % 7) / 7.0f - 0.5f);
}

/****************************************************************************
 * Mode: local — benchmark local inference
 ****************************************************************************/

static int mode_local(int n)
{
  printf("[inferd] Initializing TFLite Micro...\n");
  if (tflm_infer_init() < 0)
    {
      printf("[inferd] ERROR: tflm_infer_init failed\n");
      return -1;
    }

  printf("[inferd] Local inference benchmark  (N=%d)\n", n);
  printf("[inferd] %-6s  %-8s  %-8s\n", "run", "input", "output");

  uint32_t total_us = 0;
  float output;

  for (int i = 0; i < n; i++)
    {
      float input = synth_input(i);
      uint32_t t0 = now_us();
      tflm_infer_once(input, &output);
      uint32_t dt = now_us() - t0;
      total_us += dt;

      if (i < 5 || i == n - 1)
        printf("[inferd] %-6d  %-8.4f  %-8.4f  (%u us)\n",
               i, input, output, dt);
    }

  printf("\n[inferd] === Local Inference Stats ===\n");
  printf("[inferd] runs        : %d\n", n);
  printf("[inferd] avg latency : %.1f us  (%.3f ms)\n",
         (float)total_us / n, (float)total_us / n / 1000.0f);
  printf("[inferd] total time  : %.1f ms\n", (float)total_us / 1000.0f);
  return 0;
}

/****************************************************************************
 * Mode: server — TCP worker, accept inference tasks
 ****************************************************************************/

static int mode_server(void)
{
  printf("[inferd] Initializing TFLite Micro...\n");
  if (tflm_infer_init() < 0)
    {
      printf("[inferd] ERROR: tflm_infer_init failed\n");
      return -1;
    }

  int srv = socket(AF_INET, SOCK_STREAM, 0);
  int on  = 1;
  setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family      = AF_INET;
  addr.sin_port        = htons(INFERD_PORT);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(srv, (struct sockaddr *)&addr, sizeof(addr)) < 0 ||
      listen(srv, 4) < 0)
    {
      printf("[inferd] bind/listen failed: %s\n", strerror(errno));
      close(srv);
      return -1;
    }

  printf("[inferd] worker listening on TCP :%d\n", INFERD_PORT);
  printf("[inferd] protocol: recv 4B float → infer → send 8B (result+lat_us)\n\n");

  int total_tasks = 0;
  uint32_t total_infer_us = 0;

  for (;;)
    {
      struct sockaddr_in cli;
      socklen_t clen = sizeof(cli);
      int conn = accept(srv, (struct sockaddr *)&cli, &clen);
      if (conn < 0) continue;

      char peer[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &cli.sin_addr, peer, sizeof(peer));
      printf("[inferd] client: %s\n", peer);

      for (;;)
        {
          float input;
          int r = recv(conn, &input, sizeof(input), MSG_WAITALL);
          if (r != sizeof(float)) break;

          float output;
          uint32_t t0 = now_us();
          tflm_infer_once(input, &output);
          uint32_t lat_us = now_us() - t0;

          uint8_t resp[8];
          memcpy(resp,     &output,  4);
          memcpy(resp + 4, &lat_us,  4);
          send(conn, resp, 8, MSG_NOSIGNAL);

          total_tasks++;
          total_infer_us += lat_us;
          printf("[inferd] task #%-4d  in=%.4f  out=%.4f  %u us\n",
                 total_tasks, input, output, lat_us);
        }

      close(conn);
      printf("[inferd] %s disconnected  "
             "(total=%d  avg_infer=%.1f us)\n",
             peer, total_tasks,
             total_tasks ? (float)total_infer_us / total_tasks : 0.0f);
    }

  close(srv);
  return 0;
}

/****************************************************************************
 * Mode: bench — compare local vs offloaded latency
 ****************************************************************************/

static int mode_bench(const char *peer_ip, int n)
{
  printf("[inferd] Initializing TFLite Micro...\n");
  if (tflm_infer_init() < 0)
    {
      printf("[inferd] ERROR: tflm_infer_init failed\n");
      return -1;
    }

  /* ── Phase 1: local inference ──────────────────────────── */

  printf("[inferd] Phase 1: local inference  (N=%d)\n", n);
  uint32_t local_total_us = 0;
  float output;

  for (int i = 0; i < n; i++)
    {
      float input = synth_input(i);
      uint32_t t0 = now_us();
      tflm_infer_once(input, &output);
      local_total_us += now_us() - t0;
    }

  float local_avg_us = (float)local_total_us / n;

  /* ── Phase 2: offloaded inference ─────────────────────── */

  printf("[inferd] Phase 2: offload to %s:%d  (N=%d)\n",
         peer_ip, INFERD_PORT, n);

  int conn = socket(AF_INET, SOCK_STREAM, 0);
  struct sockaddr_in srv_addr;
  memset(&srv_addr, 0, sizeof(srv_addr));
  srv_addr.sin_family = AF_INET;
  srv_addr.sin_port   = htons(INFERD_PORT);
  inet_pton(AF_INET, peer_ip, &srv_addr.sin_addr);

  if (connect(conn, (struct sockaddr *)&srv_addr, sizeof(srv_addr)) < 0)
    {
      printf("[inferd] connect to %s failed: %s\n", peer_ip, strerror(errno));
      close(conn);
      return -1;
    }

  uint32_t offload_total_us = 0;
  int offload_fail = 0;

  for (int i = 0; i < n; i++)
    {
      float input = synth_input(i);
      uint8_t resp[8];

      uint32_t t0 = now_us();
      if (send(conn, &input, sizeof(input), MSG_NOSIGNAL) != sizeof(float))
        { offload_fail++; continue; }
      if (recv(conn, resp, 8, MSG_WAITALL) != 8)
        { offload_fail++; continue; }
      offload_total_us += now_us() - t0;
    }

  close(conn);
  float offload_avg_us = (float)offload_total_us / (n - offload_fail);

  /* ── Summary ────────────────────────────────────────────── */

  printf("\n[inferd] ╔══════════════════════════════════════╗\n");
  printf("[inferd] ║  Distributed Inference Benchmark      ║\n");
  printf("[inferd] ╠══════════════════════════════════════╣\n");
  printf("[inferd] ║  Runs          : %-5d               ║\n", n);
  printf("[inferd] ║  Peer          : %-20s ║\n", peer_ip);
  printf("[inferd] ╠══════════════════════════════════════╣\n");
  printf("[inferd] ║  Local avg     : %7.1f us  (%5.2f ms) ║\n",
         local_avg_us, local_avg_us / 1000.0f);
  printf("[inferd] ║  Offload avg   : %7.1f us  (%5.2f ms) ║\n",
         offload_avg_us, offload_avg_us / 1000.0f);
  printf("[inferd] ║  Offload fails : %-5d               ║\n", offload_fail);
  printf("[inferd] ╠══════════════════════════════════════╣\n");

  float ratio = offload_avg_us / local_avg_us;
  const char *verdict;
  if (ratio < 1.2f)
    verdict = "offload viable  (<1.2x overhead)";
  else if (ratio < 3.0f)
    verdict = "marginal — use delta-C2 scheduling";
  else
    verdict = "local preferred  (high comm cost)";

  printf("[inferd] ║  Overhead      : %-6.2fx              ║\n", ratio);
  printf("[inferd] ║  Verdict       : %-22s ║\n", verdict);
  printf("[inferd] ╚══════════════════════════════════════╝\n\n");

  return 0;
}

/****************************************************************************
 * main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  if (argc < 2)
    {
      printf("Usage:\n"
             "  inferd local  [N]         local inference benchmark\n"
             "  inferd server             TCP worker (port %d)\n"
             "  inferd bench  <ip> [N]    local vs offloaded comparison\n",
             INFERD_PORT);
      return EXIT_FAILURE;
    }

  if (strcmp(argv[1], "local") == 0)
    {
      int n = (argc >= 3) ? atoi(argv[2]) : DEFAULT_N;
      return mode_local(n) == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
    }
  else if (strcmp(argv[1], "server") == 0)
    {
      return mode_server() == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
    }
  else if (strcmp(argv[1], "bench") == 0)
    {
      if (argc < 3)
        { printf("bench: peer IP required\n"); return EXIT_FAILURE; }
      int n = (argc >= 4) ? atoi(argv[3]) : BENCH_RUNS;
      return mode_bench(argv[2], n) == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

  printf("[inferd] unknown mode: %s\n", argv[1]);
  return EXIT_FAILURE;
}
