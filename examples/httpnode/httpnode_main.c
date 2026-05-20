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
#include <fcntl.h>
#include <malloc.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include "lednode.h"

/****************************************************************************
 * Shared stats (updated by other tasks via httpnode_record_*)
 ****************************************************************************/

static volatile int   g_infer_count   = 0;
static volatile int   g_infer_x10000  = 0;  /* last result * 10000, no FPU in HTTP task */
static volatile int   g_infer_us      = 0;
static volatile int   g_peer_count    = 0;
static volatile char  g_peer_ip[INET_ADDRSTRLEN] = "-";
static volatile int   g_hb_count      = 0;
static volatile int   g_hb_fail       = 0;
static time_t         g_start_time    = 0;
static char           g_node_ip[INET_ADDRSTRLEN] = "?.?.?.?";
static volatile int   g_mem_free_kb   = 0;  /* updated by main() before accept() */
static volatile int   g_mem_total_kb  = 0;
static volatile int   g_cpu_pct       = -1; /* updated by main() before accept() */

/* Public API for other tasks */
void httpnode_record_infer(float result, int latency_us)
{
  g_infer_count++;
  g_infer_x10000 = (int)(result * 10000.0f);  /* FPU only in caller's task context */
  g_infer_us     = latency_us;
  lednode_flash(LEDNODE_INFER);
}

void httpnode_record_peer(const char *ip)
{
  g_peer_count++;
  if (ip) strncpy((char *)g_peer_ip, ip, INET_ADDRSTRLEN - 1);
}

void httpnode_record_hb(int total, int fail)
{
  int prev_fail = g_hb_fail;
  g_hb_count = total;
  g_hb_fail  = fail;
  if (fail > prev_fail)
    lednode_flash(LEDNODE_HB_FAIL);
  else
    lednode_flash(LEDNODE_HB_TX);
}

/****************************************************************************
 * Read CPU load from /proc/cpuload (file I/O — no malloc, no socket, safe)
 ****************************************************************************/

static int read_cpu_pct(void)
{
  static char buf[64] __attribute__((aligned(16)));
  int fd = open("/proc/cpuload", O_RDONLY);
  if (fd < 0) return -1;
  int r = read(fd, buf, sizeof(buf) - 1);
  close(fd);
  if (r <= 0) return -1;
  buf[r] = '\0';
  /* NuttX format: " 15.3%\n"
   * tmp = 1000 - (1000 * idle_ticks / total) → value = cpu-busy%.
   * 0.0% = system idle,  100.0% = fully loaded.  Use directly. */
  int cpu_int, cpu_frac;
  if (sscanf(buf, " %d.%d%%", &cpu_int, &cpu_frac) == 2)
    return cpu_int;
  return -1;
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
  /* CPU and memory are pre-sampled in main() BEFORE accept() (system idle).
   * Do NOT call read_cpu_pct() or mallinfo() here — measuring inside the
   * request handler captures peak load and gives a bogus 100% reading. */
  int cpu_pct = g_cpu_pct;
  int cpu_bar = (cpu_pct >= 0 && cpu_pct <= 100) ? cpu_pct : 0;
  char cpu_str[10];
  if (cpu_pct < 0) strncpy(cpu_str, "N/A", sizeof(cpu_str));
  else snprintf(cpu_str, sizeof(cpu_str), "%d%%", cpu_pct);

  int mem_free  = g_mem_free_kb;
  int mem_total = g_mem_total_kb;
  int mem_bar   = mem_total > 0 ? (100 * (mem_total - mem_free) / mem_total) : 0;

  lednode_flash(LEDNODE_HTTP_REQ);

  /* Drain the incoming HTTP request — required to flush lwIP IOBs before
   * calling send().  Without this, send() processes stale IOBs that are
   * not PSRAM-aligned, triggering EXCCAUSE=3 (LoadStoreAlignmentCause).
   * Static + aligned(16) avoids the same trap on the recv side.
   * NOTE: never call get_ip() / socket() here — opening a socket fd
   * inside handle_request crashes up_saveusercontext on S3. */
  static char req[256] __attribute__((aligned(16)));
  recv(conn, req, sizeof(req) - 1, 0);

  /* Uptime from recorded start time, not boot time */
  long uptime_s = (long)(time(NULL) - g_start_time);
  if (uptime_s < 0) uptime_s = 0;
  long up_h = uptime_s / 3600;
  long up_m = (uptime_s % 3600) / 60;
  long up_s = uptime_s % 60;

  /* Pre-format floats — avoid %f in big snprintf (dtoa mutex → up_saveusercontext crash) */
  int rel_pct = g_hb_count
    ? (int)(100 * (g_hb_count - g_hb_fail) / g_hb_count)
    : 100;
  const char *rel_cls = g_hb_count == 0    ? "g" :
                        rel_pct >= 90      ? "g" :
                        rel_pct >= 70      ? "y" : "rr";

  char score_str[20];
  {
    int sv    = g_infer_x10000;
    int s_int  = sv / 10000;
    int s_frac = sv % 10000;
    if (s_frac < 0) { s_frac = -s_frac; s_int = -s_int; }
    snprintf(score_str, sizeof(score_str), "%d.%04d", s_int, s_frac);
  }

  /* Latency display */
  char lat_str[20];
  if (g_infer_us == 0)
    snprintf(lat_str, sizeof(lat_str), "&lt;1 µs");
  else if (g_infer_us < 1000)
    snprintf(lat_str, sizeof(lat_str), "%d µs", g_infer_us);
  else
    snprintf(lat_str, sizeof(lat_str), "%d.%02d ms",
             g_infer_us / 1000, (g_infer_us % 1000) / 10);


  /* ── Build HTML ──────────────────────────────────────── */

  static char body[4096] __attribute__((aligned(16)));  /* 16B align for PSRAM lwIP send */
  int n = snprintf(body, sizeof(body),
    "<!DOCTYPE html><html><head>"
    "<meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<meta http-equiv='refresh' content='3'>"
    "<title>openvela S3 Node</title>"
    "<style>"
    "*{box-sizing:border-box;margin:0;padding:0}"
    "body{font-family:sans-serif;background:#f0f2f5;color:#1f2937;padding:12px}"
    "h2{background:linear-gradient(135deg,#6366f1,#8b5cf6);color:#fff;"
       "border-radius:12px;padding:14px;margin-bottom:10px;font-size:1em}"
    "h2 small{font-size:.78em;opacity:.85;font-weight:400}"
    ".c{background:#fff;border-radius:12px;padding:12px;margin:8px 0;"
       "box-shadow:0 1px 4px rgba(0,0,0,.08)}"
    ".t{font-size:.7em;font-weight:700;color:#6b7280;text-transform:uppercase;letter-spacing:.06em}"
    ".v{font-size:1.55em;font-weight:700;margin:4px 0;color:#111827}"
    ".r{display:flex;justify-content:space-between;align-items:center;"
       "font-size:.82em;color:#374151;margin-top:5px}"
    ".g{color:#166534;background:#dcfce7;padding:2px 10px;border-radius:12px;font-weight:700}"
    ".y{color:#854d0e;background:#fef9c3;padding:2px 10px;border-radius:12px;font-weight:700}"
    ".rr{color:#991b1b;background:#fee2e2;padding:2px 10px;border-radius:12px;font-weight:700}"
    ".b{height:6px;border-radius:3px;background:#e5e7eb;margin:5px 0 2px}"
    ".f{height:6px;border-radius:3px}"
    "footer{text-align:center;font-size:.72em;color:#9ca3af;margin-top:12px}"
    "a{color:#6366f1}"
    "</style></head><body>"

    "<h2>&#x1F4E1; openvela ESP32-S3<br><small>%s &nbsp;|&nbsp; up %ldh%ldm%lds</small></h2>"

    "<div class='c'><div class='t'>AI Inference</div>"
    "<div class='v'>%d calls</div>"
    "<div class='r'><span>Score</span><span><b>%s</b></span></div>"
    "<div class='r'><span>Latency</span><span><b>%s</b></span></div></div>"

    "<div class='c'><div class='t'>Heartbeat</div>"
    "<div class='v'><span class='%s'>%d%%</span></div>"
    "<div class='r'><span>TX</span><span>%d</span></div>"
    "<div class='r'><span>Fail</span><span>%d</span></div></div>"

    "<div class='c'><div class='t'>Peers</div>"
    "<div class='v'>%d</div>"
    "<div class='r'><span>Last seen</span><span>%s</span></div></div>"

    "<div class='c'><div class='t'>&#x1F4BB; System</div>"
    "<div class='r'><span>CPU</span><span><b>%s</b></span></div>"
    "<div class='b'><div class='f' style='width:%d%%;background:#6366f1'></div></div>"
    "<div class='r'><span>RAM free</span><span><b>%d / %d KB</b></span></div>"
    "<div class='b'><div class='f' style='width:%d%%;background:#f59e0b'></div></div></div>"

    "<footer>auto-refresh 3s &bull; <a href='/'>reload</a></footer>"
    "</body></html>",

    g_node_ip, up_h, up_m, up_s,
    g_infer_count, score_str, lat_str,
    rel_cls, rel_pct,
    g_hb_count, g_hb_fail,
    g_peer_count, (char *)g_peer_ip,
    cpu_str, cpu_bar,
    mem_free, mem_total, mem_bar
  );

  /* Clamp n to actual buffer size */
  if (n <= 0 || n >= (int)sizeof(body)) n = (int)sizeof(body) - 1;

  char hdr[256] __attribute__((aligned(4)));
  snprintf(hdr, sizeof(hdr),
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/html; charset=utf-8\r\n"
    "Content-Length: %d\r\n"
    "Connection: close\r\n\r\n", n);

  send(conn, hdr, strlen(hdr), 0);
  send(conn, body, n, 0);
  close(conn);
}

/****************************************************************************
 * main — run as background job: "httpnode &"
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int port = (argc > 1) ? atoi(argv[1]) : 8080;

  g_start_time = time(NULL);   /* record start for uptime calculation */
  printf("[http] openvela HTTP node dashboard  (port %d)\n", port);
  printf("[http] tip: run as  'httpnode &'  to background\n");

  int srv = socket(AF_INET, SOCK_STREAM, 0);
  int on  = 1;
  setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family      = AF_INET;
  addr.sin_port        = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(srv, (struct sockaddr *)&addr, sizeof(addr)) < 0 ||
      listen(srv, 4) < 0)
    {
      printf("[http] bind/listen on port %d failed: %s\n",
             port, strerror(errno));
      close(srv);
      return EXIT_FAILURE;
    }

  get_ip(g_node_ip, sizeof(g_node_ip));
  printf("[http] ✓ serving at http://%s:%d/\n", g_node_ip, port);
  printf("[http] open in phone browser (same WiFi: 312)\n");

  for (;;)
    {
      /* Sample CPU and memory BEFORE blocking on accept() — system is idle
       * here so /proc/cpuload reflects true background load, not our own
       * request-handling overhead.  mallinfo() is safe at this stack depth. */
      g_cpu_pct = read_cpu_pct();
      {
        struct mallinfo mi = mallinfo();
        g_mem_total_kb = mi.arena / 1024;
        g_mem_free_kb  = (mi.arena - mi.uordblks) / 1024;
      }

      int conn = accept(srv, NULL, NULL);  /* blocks until browser connects */
      if (conn >= 0) handle_request(conn);
    }

  close(srv);
  return EXIT_SUCCESS;
}
