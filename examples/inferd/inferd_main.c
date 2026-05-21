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
#include <fcntl.h>

#include "lednode.h"
#include "infer_rpc.h"

/* httpnode stats hook — weak so inferd works without httpnode running */
extern void httpnode_record_infer(float result, int latency_us)
  __attribute__((weak));

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

#define DEFAULT_N         100
#define BENCH_RUNS        50
#define BATCH_SIZE        1000   /* repeat each call N times for stable timing */

/****************************************************************************
 * δ-C2 peer state  (extern declared in infer_rpc.h)
 ****************************************************************************/

dc2_peer_t   g_dc2_peers[DC2_MAX_PEERS];
volatile int g_dc2_n_peers = 0;

/****************************************************************************
 * Helpers
 ****************************************************************************/

static float read_cpu_pct(void)
{
  float pct = 0.0f;
#ifdef CONFIG_FS_PROCFS
  int fd = open("/proc/cpuload", O_RDONLY);
  if (fd >= 0)
    {
      char buf[32];
      int n = read(fd, buf, sizeof(buf) - 1);
      close(fd);
      if (n > 0) { buf[n] = '\0'; pct = strtof(buf, NULL); }
    }
#endif
  return pct;
}

dc2_decision_t dc2_decide(float local_cpu, const char **peer_ip_out)
{
  *peer_ip_out = NULL;
  if (local_cpu < DC2_LOCAL_CPU_THRESH || g_dc2_n_peers == 0)
    return DC2_LOCAL;

  float best = 200.0f;
  int   bidx = -1;
  for (int i = 0; i < g_dc2_n_peers; i++)
    {
      if (g_dc2_peers[i].ip[0] == '\0') continue;
      float c = g_dc2_peers[i].cpu_pct < 0 ? 50.0f : g_dc2_peers[i].cpu_pct;
      if (c < best) { best = c; bidx = i; }
    }

  if (bidx < 0) return DC2_LOCAL;
  *peer_ip_out = g_dc2_peers[bidx].ip;
  return DC2_OFFLOAD;
}

void dc2_update_peer(const char *ip, float cpu_pct)
{
  for (int i = 0; i < g_dc2_n_peers; i++)
    {
      if (strncmp(g_dc2_peers[i].ip, ip, 15) == 0)
        {
          g_dc2_peers[i].cpu_pct = cpu_pct;
          return;
        }
    }
  if (g_dc2_n_peers < DC2_MAX_PEERS)
    {
      strncpy(g_dc2_peers[g_dc2_n_peers].ip, ip, 15);
      g_dc2_peers[g_dc2_n_peers].cpu_pct = cpu_pct;
      g_dc2_n_peers++;
    }
}

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
 * Persistent offload connection helpers
 ****************************************************************************/

static int  s_offload_fd = -1;
static char s_offload_ip[16];

static int ensure_offload_conn(const char *ip)
{
  if (s_offload_fd >= 0 && strncmp(s_offload_ip, ip, 15) == 0)
    return s_offload_fd;

  if (s_offload_fd >= 0) { close(s_offload_fd); s_offload_fd = -1; }

  int conn = socket(AF_INET, SOCK_STREAM, 0);
  if (conn < 0) return -1;

  struct timeval tv;
  tv.tv_sec  = DC2_OFFLOAD_TO_MS / 1000;
  tv.tv_usec = (DC2_OFFLOAD_TO_MS % 1000) * 1000;
  setsockopt(conn, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(conn, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  struct sockaddr_in srv;
  memset(&srv, 0, sizeof(srv));
  srv.sin_family = AF_INET;
  srv.sin_port   = htons(INFERD_PORT);
  inet_pton(AF_INET, ip, &srv.sin_addr);

  if (connect(conn, (struct sockaddr *)&srv, sizeof(srv)) < 0)
    { close(conn); return -1; }

  s_offload_fd = conn;
  strncpy(s_offload_ip, ip, 15);
  printf("[inferd] δ-C2: connected to offload peer %s:%d\n", ip, INFERD_PORT);
  return conn;
}

static int do_offload(const char *ip, float input,
                      float *output, uint32_t *lat_us_out)
{
  int conn = ensure_offload_conn(ip);
  if (conn < 0) return 0;

  uint32_t t0 = now_us();

  if (send(conn, &input, sizeof(input), MSG_NOSIGNAL) != (int)sizeof(float))
    goto fail;

  uint8_t resp[8];
  if (recv(conn, resp, 8, MSG_WAITALL) != 8)
    goto fail;

  *lat_us_out = now_us() - t0;
  memcpy(output, resp, sizeof(float));
  return 1;

fail:
  close(conn);
  s_offload_fd = -1;
  s_offload_ip[0] = '\0';
  return 0;
}

/****************************************************************************
 * Mode: daemon — δ-C2 adaptive inference loop
 ****************************************************************************/

static int mode_daemon(int n, int interval_ms)
{
  if (tflm_infer_init() < 0)
    {
      fprintf(stderr, "[inferd] tflm_infer_init failed\n");
      return -1;
    }

  printf("[inferd] δ-C2 daemon  n=%s  interval=%dms  "
         "offload_thresh=%.0f%%\n",
         n == 0 ? "inf" : "", interval_ms,
         (double)DC2_LOCAL_CPU_THRESH);
  printf("[inferd] peers registered: %d\n\n", g_dc2_n_peers);

  int      cnt_local    = 0;
  int      cnt_offload  = 0;
  int      cnt_fallback = 0;
  uint32_t sum_local_us   = 0;
  uint32_t sum_offload_us = 0;

  for (int i = 0; n == 0 || i < n; i++)
    {
      float    input  = synth_input(i);
      float    output = 0.0f;
      uint32_t lat_us = 0;
      const char *path_label;

      float       local_cpu = read_cpu_pct();
      const char *peer_ip   = NULL;
      dc2_decision_t dec    = dc2_decide(local_cpu, &peer_ip);

      if (dec == DC2_OFFLOAD && peer_ip != NULL)
        {
          if (do_offload(peer_ip, input, &output, &lat_us))
            {
              cnt_offload++;
              sum_offload_us += lat_us;
              path_label = "offload";
              lednode_flash(LEDNODE_HB_TX);
            }
          else
            {
              uint32_t t0 = now_us();
              tflm_infer_once(input, &output);
              lat_us = now_us() - t0;
              cnt_fallback++;
              sum_local_us += lat_us;
              path_label = "fallback";
              lednode_flash(LEDNODE_INFER);
            }
        }
      else
        {
          uint32_t t0 = now_us();
          tflm_infer_once(input, &output);
          lat_us = now_us() - t0;
          cnt_local++;
          sum_local_us += lat_us;
          path_label = "local";
          lednode_flash(LEDNODE_INFER);
        }

      if (httpnode_record_infer)
        httpnode_record_infer(output, (int)lat_us);

      printf("[inferd] #%-4d  cpu=%5.1f%%  %-8s  peer=%-15s  "
             "in=%.4f  out=%.4f  %5u us\n",
             i, (double)local_cpu, path_label,
             peer_ip ? peer_ip : "-", (double)input,
             (double)output, lat_us);

      usleep(interval_ms * 1000);
    }

  int total_off = cnt_offload + cnt_fallback;
  printf("\n[inferd] ── δ-C2 Daemon Summary ──────────────────\n");
  printf("[inferd] local     : %d  (avg %.1f us)\n",
         cnt_local,
         cnt_local ? (double)sum_local_us / cnt_local : 0.0);
  printf("[inferd] offload   : %d  (avg %.1f us)\n",
         cnt_offload,
         cnt_offload ? (double)sum_offload_us / cnt_offload : 0.0);
  printf("[inferd] fallback  : %d\n", cnt_fallback);
  if (total_off > 0)
    printf("[inferd] offload success rate: %.1f%%\n",
           100.0 * cnt_offload / total_off);
  printf("[inferd] ──────────────────────────────────────────\n\n");
  return 0;
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

      /* Batch BATCH_SIZE calls to get stable timing on fast mock */
      uint32_t t0 = now_us();
      for (int b = 0; b < BATCH_SIZE; b++)
        tflm_infer_once(input, &output);
      uint32_t dt_batch = now_us() - t0;
      uint32_t dt = dt_batch / BATCH_SIZE;   /* per-call latency */
      total_us += dt;

      if (httpnode_record_infer)
        httpnode_record_infer(output, (int)(dt ? dt : dt_batch));

      if (i < 5 || i == n - 1)
        printf("[inferd] %-6d  %-8.4f  %-8.4f  (%u us)\n",
               i, input, output, dt ? dt : dt_batch);
    }

  printf("\n[inferd] === Local Inference Stats ===\n");
#ifdef CONFIG_EXAMPLES_INFERD_TFLITE
  printf("[inferd] engine      : TFLite Micro (real)\n");
#else
  printf("[inferd] engine      : mock sinf  (x%d batch avg)\n", BATCH_SIZE);
#endif
  printf("[inferd] runs        : %d\n", n);
  printf("[inferd] avg latency : %.2f us  (%.4f ms)\n",
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
 * Mode: stress — heavy FPU load, makes CPU visibly busy in dashboard
 *
 * Each call computes 50 000 iterations of sinf/cosf/sqrtf — enough to
 * run for several ms on LX7@240MHz.  Latency shows as ms in dashboard.
 ****************************************************************************/

static int mode_stress(int interval_ms)
{
  const int N = 50000;
  printf("[inferd] stress mode  N=%d FPU ops/call  interval=%dms\n", N, interval_ms);
  printf("[inferd] watch CPU load rise at http://<ip>:8080/\n");
  lednode_base(LEDNODE_STRESS);   /* orange solid while stress runs */
  int count = 0;
  for (;;)
    {
      float acc = 0.0f;
      uint32_t t0 = now_us();
      for (int i = 1; i <= N; i++)
        acc += sinf((float)i * 0.001f) * cosf((float)i * 0.0007f)
             + sqrtf((float)i * 0.0001f);
      int dt = (int)(now_us() - t0);
      count++;
      float result = acc / N;   /* normalised to ~0-1 */
      if (httpnode_record_infer)
        httpnode_record_infer(result, dt);
      printf("[inferd] stress #%-4d  acc=%+.4f  %d us (%d ms)\n",
             count, result, dt, dt / 1000);
      usleep(interval_ms * 1000);
    }
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
             "  inferd local  [N]               local inference benchmark\n"
             "  inferd demo   [interval_ms]     continuous demo (default 1000ms)\n"
             "  inferd stress [interval_ms]     heavy FPU load (shows CPU in dashboard)\n"
             "  inferd server                   TCP worker (port %d)\n"
             "  inferd bench  <ip> [N]          local vs offloaded comparison\n"
             "  inferd daemon [N [interval_ms]] delta-C2 adaptive daemon\n"
             "    N=0 runs forever; interval default 500ms\n",
             INFERD_PORT);
      return EXIT_FAILURE;
    }

  if (strcmp(argv[1], "local") == 0)
    {
      int n = (argc >= 3) ? atoi(argv[2]) : DEFAULT_N;
      return mode_local(n) == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
    }
  else if (strcmp(argv[1], "demo") == 0)
    {
      int interval_ms = (argc >= 3) ? atoi(argv[2]) : 1000;
      if (tflm_infer_init() < 0) return EXIT_FAILURE;
      printf("[inferd] demo mode  interval=%dms  Ctrl+C to stop\n",
             interval_ms);
      printf("[inferd] watch dashboard at http://<ip>:8080/\n");
      int count = 0;
      for (;;)
        {
          float input = synth_input(count);
          float output;
          uint32_t t0 = now_us();
          tflm_infer_once(input, &output);
          int dt = (int)(now_us() - t0);
          count++;
          if (httpnode_record_infer)
            httpnode_record_infer(output, dt);
          printf("[inferd] demo #%-4d  in=%.4f  out=%.4f  %dus\n",
                 count, input, output, dt);
          usleep(interval_ms * 1000);
        }
      return EXIT_SUCCESS;
    }
  else if (strcmp(argv[1], "stress") == 0)
    {
      int interval_ms = (argc >= 3) ? atoi(argv[2]) : 1000;
      return mode_stress(interval_ms);
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

  else if (strcmp(argv[1], "daemon") == 0)
    {
      int n   = (argc >= 3) ? atoi(argv[2]) : 0;
      int ms  = (argc >= 4) ? atoi(argv[3]) : 500;
      return mode_daemon(n, ms) == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

  printf("[inferd] unknown mode: %s\n", argv[1]);
  return EXIT_FAILURE;
}
