/****************************************************************************
 * apps/examples/httpnode/httpnode_main.c
 *
 * Minimal HTTP status page for openvela ESP32-S3 mesh nodes.
 * No filesystem required — HTML is inlined.
 *
 * Usage:  httpnode [port]   (default port 80)
 *
 * Open http://<S3-IP>/ from a phone browser on the same WiFi.
 * Page auto-refreshes every 3 seconds showing:
 *   - Node IP, uptime
 *   - Inference stats (calls to increment via httpnode_record_infer)
 *   - Peer discovery status
 *   - Heartbeat count from nodelink_daemon
 *
 * Build: CONFIG_EXAMPLES_HTTPNODE=y
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <sys/ioctl.h>

/****************************************************************************
 * Shared stats (updated by other tasks via httpnode_record_*)
 ****************************************************************************/

static volatile int   g_infer_count   = 0;
static volatile float g_infer_last    = 0.0f;
static volatile int   g_infer_us      = 0;
static volatile int   g_peer_count    = 0;
static volatile char  g_peer_ip[INET_ADDRSTRLEN] = "—";
static volatile int   g_hb_count      = 0;
static volatile int   g_hb_fail       = 0;

/* Public API for other tasks */
void httpnode_record_infer(float result, int latency_us)
{
  g_infer_count++;
  g_infer_last = result;
  g_infer_us   = latency_us;
}

void httpnode_record_peer(const char *ip)
{
  g_peer_count++;
  if (ip) strncpy((char *)g_peer_ip, ip, INET_ADDRSTRLEN - 1);
}

void httpnode_record_hb(int total, int fail)
{
  g_hb_count = total;
  g_hb_fail  = fail;
}

/****************************************************************************
 * Get own IP
 ****************************************************************************/

static void get_ip(char *buf, size_t len)
{
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) { strncpy(buf, "?.?.?.?", len); return; }
  struct ifreq ifr;
  strncpy(ifr.ifr_name, "wlan0", IFNAMSIZ);
  if (ioctl(fd, SIOCGIFADDR, &ifr) == 0)
    inet_ntop(AF_INET,
              &((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr,
              buf, len);
  else
    strncpy(buf, "?.?.?.?", len);
  close(fd);
}

/****************************************************************************
 * Handle one HTTP request
 ****************************************************************************/

static void handle_request(int conn)
{
  char req[256];
  recv(conn, req, sizeof(req) - 1, 0);   /* read and discard */

  char ip[INET_ADDRSTRLEN];
  get_ip(ip, sizeof(ip));

  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  long uptime_s = ts.tv_sec;

  float reliability = g_hb_count
    ? 100.0f * (float)(g_hb_count - g_hb_fail) / (float)g_hb_count
    : 100.0f;

  /* ── Build HTML ──────────────────────────────────────── */

  char body[2048];
  int n = snprintf(body, sizeof(body),
    "<!DOCTYPE html><html><head>"
    "<meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<meta http-equiv='refresh' content='3'>"
    "<title>openvela S3 Node</title>"
    "<style>"
    "body{font-family:monospace;background:#0d1117;color:#e6edf3;margin:20px}"
    "h1{color:#58a6ff;font-size:1.2em}"
    ".card{background:#161b22;border:1px solid #30363d;border-radius:8px;"
           "padding:12px;margin:10px 0}"
    ".label{color:#8b949e;font-size:.85em}"
    ".val{color:#3fb950;font-size:1.1em;font-weight:bold}"
    ".warn{color:#f78166}"
    ".ok{color:#3fb950}"
    "</style></head><body>"
    "<h1>&#x1F4E1; openvela ESP32-S3 Node</h1>"
    "<div class='card'>"
    "<div class='label'>IP Address</div>"
    "<div class='val'>%s</div>"
    "<div class='label'>Uptime</div>"
    "<div class='val'>%ldm %lds</div>"
    "</div>"
    "<div class='card'>"
    "<div class='label'>AI Inference</div>"
    "<div class='val'>%d calls &nbsp; last=%.4f &nbsp; %d us</div>"
    "</div>"
    "<div class='card'>"
    "<div class='label'>Peers Discovered</div>"
    "<div class='val'>%d &nbsp; last: %s</div>"
    "</div>"
    "<div class='card'>"
    "<div class='label'>Heartbeat</div>"
    "<div class='val'>tx=%d fail=%d reliability=<span class='%s'>%.1f%%</span></div>"
    "</div>"
    "<p style='color:#8b949e;font-size:.75em'>auto-refresh 3s &bull; "
    "<a href='/' style='color:#58a6ff'>reload</a></p>"
    "</body></html>",
    ip,
    uptime_s / 60, uptime_s % 60,
    g_infer_count, g_infer_last, g_infer_us,
    g_peer_count, (char *)g_peer_ip,
    g_hb_count, g_hb_fail,
    reliability >= 90.0f ? "ok" : "warn",
    reliability);

  char hdr[256];
  snprintf(hdr, sizeof(hdr),
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/html; charset=utf-8\r\n"
    "Content-Length: %d\r\n"
    "Connection: close\r\n\r\n", n);

  send(conn, hdr,  strlen(hdr), MSG_NOSIGNAL);
  send(conn, body, n,           MSG_NOSIGNAL);
  close(conn);
}

/****************************************************************************
 * Server thread
 ****************************************************************************/

static void *server_thread(void *arg)
{
  int port = (int)(intptr_t)arg;
  int srv  = socket(AF_INET, SOCK_STREAM, 0);
  int on   = 1;
  setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family      = AF_INET;
  addr.sin_port        = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(srv, (struct sockaddr *)&addr, sizeof(addr)) < 0 ||
      listen(srv, 4) < 0)
    {
      printf("[http] bind/listen failed: %s\n", strerror(errno));
      close(srv);
      return NULL;
    }

  char ip[INET_ADDRSTRLEN];
  get_ip(ip, sizeof(ip));
  printf("[http] serving at http://%s:%d/  (auto-refresh 3s)\n", ip, port);

  for (;;)
    {
      int conn = accept(srv, NULL, NULL);
      if (conn >= 0) handle_request(conn);
    }

  return NULL;
}

/****************************************************************************
 * main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int port = (argc > 1) ? atoi(argv[1]) : 80;

  printf("[http] openvela HTTP node dashboard  (port %d)\n", port);

  pthread_t tid;
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setstacksize(&attr, 4096);
  pthread_create(&tid, &attr, server_thread, (void *)(intptr_t)port);
  pthread_attr_destroy(&attr);
  pthread_detach(tid);

  printf("[http] background server started, returning to NSH\n");
  return EXIT_SUCCESS;
}
