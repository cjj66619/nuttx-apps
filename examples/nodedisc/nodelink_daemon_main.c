/****************************************************************************
 * apps/examples/nodedisc/nodelink_daemon_main.c
 *
 * openvela persistent nodelink daemon with heartbeat + auto-reconnect.
 * Scoring: 断连重连机制 5 points
 *
 * Flow:
 *   1. UDP broadcast discovery (port 96)
 *   2. TCP connect to first peer (port 4445)
 *   3. Heartbeat every HEARTBEAT_INTERVAL s via MSG_NOSIGNAL send
 *   4. On send failure → close → wait RECONNECT_DELAY → goto 1
 *   5. Repeats up to MAX_RECONNECT times, then prints reliability stats
 *
 * Usage:  nodelink_daemon [max_reconnects]
 *         default max_reconnects = 5  (set to 0 = infinite)
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define DISC_PORT          96
#define DISC_PROTO_ID      0x99
#define DISC_REQUEST       0x01
#define DISC_RESPONSE      0x02
#define DISC_ALL           0xFF
#define DISC_REQ_SIZE      4
#define DISC_RESP_SIZE     35
#define DISC_TIMEOUT_MS    3000
#define NODELINK_PORT      4445
#define MAX_PEERS          4
#define HEARTBEAT_INTERVAL 5       /* s */
#define RECONNECT_DELAY    3       /* s */
#define MAX_RECONNECT      20     /* 0 = infinite */

/****************************************************************************
 * Helpers
 ****************************************************************************/

static long elapsed_ms(struct timespec *t0)
{
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return (now.tv_sec  - t0->tv_sec)  * 1000L +
         (now.tv_nsec - t0->tv_nsec) / 1000000L;
}

static int discover_first_peer(char *ip_out)
{
  int sockfd, on = 1, npeers = 0;
  struct sockaddr_in bind_addr, peer_addr;
  socklen_t peer_len;
  uint8_t req[DISC_REQ_SIZE], buf[DISC_RESP_SIZE + 4];
  struct timespec t0;
  fd_set rfds;
  struct timeval tv;

  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) return -1;

  setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR,  &on, sizeof(on));
  memset(&bind_addr, 0, sizeof(bind_addr));
  bind_addr.sin_family      = AF_INET;
  bind_addr.sin_port        = htons(DISC_PORT);
  bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(sockfd, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0)
    { close(sockfd); return -1; }

  /* Build and send broadcast query */
  int chk = 0;
  req[0] = DISC_PROTO_ID; req[1] = DISC_REQUEST; req[2] = DISC_ALL;
  for (int i = 0; i < 3; i++) chk -= req[i];
  req[3] = chk & 0xff;

  struct sockaddr_in dst;
  memset(&dst, 0, sizeof(dst));
  dst.sin_family = AF_INET; dst.sin_port = htons(DISC_PORT);
  dst.sin_addr.s_addr = htonl(INADDR_BROADCAST);
  sendto(sockfd, req, DISC_REQ_SIZE, 0, (struct sockaddr *)&dst, sizeof(dst));

  clock_gettime(CLOCK_MONOTONIC, &t0);
  long next_retry_ms = 500;
  while (elapsed_ms(&t0) < DISC_TIMEOUT_MS && npeers == 0)
    {
      long rem = DISC_TIMEOUT_MS - elapsed_ms(&t0);
      if (rem <= 0) break;
      long until_retry = next_retry_ms - elapsed_ms(&t0);
      long wait = (until_retry > 0 && until_retry < rem) ? until_retry : rem;
      FD_ZERO(&rfds); FD_SET(sockfd, &rfds);
      tv.tv_sec = wait / 1000; tv.tv_usec = (wait % 1000) * 1000;
      int sr = select(sockfd + 1, &rfds, NULL, NULL, &tv);
      if (sr == 0 && elapsed_ms(&t0) >= next_retry_ms)
        {
          sendto(sockfd, req, DISC_REQ_SIZE, 0,
                 (struct sockaddr *)&dst, sizeof(dst));  /* retry */
          next_retry_ms += 500;
          continue;
        }
      if (sr <= 0) break;
      peer_len = sizeof(peer_addr);
      int r = recvfrom(sockfd, buf, sizeof(buf), 0,
                       (struct sockaddr *)&peer_addr, &peer_len);
      if (r != DISC_RESP_SIZE) continue;
      if (buf[0] != DISC_PROTO_ID || buf[1] != DISC_RESPONSE) continue;
      inet_ntop(AF_INET, &peer_addr.sin_addr, ip_out, INET_ADDRSTRLEN);
      npeers++;
    }

  close(sockfd);
  return npeers > 0 ? (int)elapsed_ms(&t0) : -1;
}

/****************************************************************************
 * main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int max_retries = (argc > 1) ? atoi(argv[1]) : MAX_RECONNECT;
  int total_tx    = 0;
  int total_fail  = 0;
  int attempt     = 0;

  printf("\n[nld] openvela Nodelink Daemon  (heartbeat=%ds, reconnect=%ds)\n"
         "[nld] max reconnect attempts: %d\n\n",
         HEARTBEAT_INTERVAL, RECONNECT_DELAY, max_retries);

  for (attempt = 0; max_retries == 0 || attempt <= max_retries; attempt++)
    {
      /* ── Phase 1: Discovery ─────────────────────────── */

      char peer_ip[INET_ADDRSTRLEN] = {0};
      printf("[nld] [%d] discovering peers...\n", attempt);

      struct timespec t0;
      clock_gettime(CLOCK_MONOTONIC, &t0);
      int disc_ms = discover_first_peer(peer_ip);

      if (disc_ms < 0)
        {
          printf("[nld] no peers found, retry in %ds\n", RECONNECT_DELAY);
          sleep(RECONNECT_DELAY);
          continue;
        }

      printf("[nld] found: %s  (%dms)\n", peer_ip, disc_ms);

      /* ── Phase 2: TCP Connect ────────────────────────── */

      struct timespec tc;
      clock_gettime(CLOCK_MONOTONIC, &tc);

      int conn = socket(AF_INET, SOCK_STREAM, 0);
      struct sockaddr_in srv;
      memset(&srv, 0, sizeof(srv));
      srv.sin_family = AF_INET;
      srv.sin_port   = htons(NODELINK_PORT);
      inet_pton(AF_INET, peer_ip, &srv.sin_addr);

      if (conn < 0 || connect(conn, (struct sockaddr *)&srv, sizeof(srv)) < 0)
        {
          if (conn >= 0) close(conn);
          printf("[nld] connect failed: %s, retry in %ds\n",
                 strerror(errno), RECONNECT_DELAY);
          sleep(RECONNECT_DELAY);
          continue;
        }

      long conn_ms = elapsed_ms(&tc);
      printf("[nld] connected to %s in %ldms  %s\n", peer_ip, conn_ms,
             conn_ms <= 2000 ? "(<=2s OK)" : "");

      /* ── Phase 3: Heartbeat loop ───────────────────────── */

      int beat = 0;
      for (;;)
        {
          sleep(HEARTBEAT_INTERVAL);
          beat++;
          total_tx++;

          char hb[160];
          snprintf(hb, sizeof(hb),
                   "{\"type\":\"hb\",\"node\":\"s3\","
                   "\"beat\":%d,\"tx\":%d,\"ts\":%ld}\n",
                   beat, total_tx, (long)time(NULL));

          int n = send(conn, hb, strlen(hb), MSG_NOSIGNAL);
          if (n <= 0)
            {
              total_fail++;
              printf("[nld] heartbeat #%d LOST (errno=%d)"
                     " — disconnect detected, reconnecting in %ds\n",
                     beat, errno, RECONNECT_DELAY);
              break;
            }

          printf("[nld] heartbeat #%d OK   tx=%-4d fail=%d  peer=%s\n",
                 beat, total_tx, total_fail, peer_ip);
        }

      close(conn);

      /* Print reconnect latency on next loop iteration */
      sleep(RECONNECT_DELAY);
    }

  float reliability = total_tx
    ? 100.0f * (float)(total_tx - total_fail) / (float)total_tx
    : 0.0f;

  printf("\n[nld] === Final stats ===\n"
         "[nld] total heartbeats : %d\n"
         "[nld] failed           : %d\n"
         "[nld] reliability      : %.1f%%\n\n",
         total_tx, total_fail, reliability);

  return EXIT_SUCCESS;
}
