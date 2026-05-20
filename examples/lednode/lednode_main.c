/****************************************************************************
 * apps/examples/lednode/lednode_main.c
 *
 * RGB LED feedback daemon for openvela ESP32-S3 distributed AI node.
 *
 * Hardware: WS2812 × 1 on GPIO48 (DevKitC v1.0) or GPIO38 (older).
 * Interface: /dev/rmt0  (RMT channel 0, registered at bringup with
 *            CONFIG_ESP_RMT=y and CONFIG_RMTCHAR=y).
 *
 * Run:  nsh> lednode &
 *
 * Other tasks signal events by writing to the shared globals declared in
 * lednode.h.  lednode polls at 50ms (20Hz) — enough for smooth animation.
 *
 * Animation engine
 * ─────────────────
 *  g_led_base  : persistent state (BOOT → WIFI_CONN → IDLE or STRESS)
 *  g_led_flash : timed override event (-1 = none).  Auto-expires after
 *                the event-specific tick count.  Higher-valued events have
 *                higher priority; a new event replaces a running one only
 *                if its priority is ≥ the current flash's priority.
 *
 * WiFi auto-detect
 * ─────────────────
 *  Every 2 s lednode polls wlan0 via ioctl(SIOCGIFADDR).  It transitions
 *  automatically:  no-IP → WIFI_CONN base,  IP present → IDLE base.
 *  The STRESS base is set externally (inferd) and not overridden by WiFi.
 *
 * WS2812 encoding
 * ────────────────
 *  WS2812 = GRB bit-order.  Each bit is one 32-bit RMT word:
 *    bit=1 : (T1L << 16) | 0x8000 | T1H
 *    bit=0 : (T0L << 16) | 0x8000 | T0H
 *  Timings at APB 80 MHz (12.5 ns/tick): T0H=28, T0L=72, T1H=72, T1L=28.
 *  24 data words + 1 zero reset word = 100 bytes per frame.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdint.h>

#include "lednode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RMT_DEV            "/dev/rmt0"
#define TICK_MS            50          /* 20 Hz animation tick */
#define WIFI_CHECK_TICKS   40          /* check WiFi every 2 s */
#define LED_BRIGHTNESS     40          /* global scale 0-255 */

/* WS2812 timing at APB 80 MHz (12.5 ns per count) */
#define APB_NS             12.5f
#define NS_TO_APB(ns)      ((uint16_t)((ns) / APB_NS))
#define T0H                NS_TO_APB(350)
#define T0L                NS_TO_APB(900)
#define T1H                NS_TO_APB(900)
#define T1L                NS_TO_APB(350)

/* RMT frame: 24 data words (3 bytes × 8 bits) + 1 reset word */
#define RMT_DATA_WORDS     24
#define RMT_TOTAL_WORDS    (RMT_DATA_WORDS + 1)

/* Flash-event ticks table (indexed by event id) */
static const int8_t k_flash_ticks[] =
{
  0,                      /* 0  LEDNODE_BOOT      (not a flash) */
  0,                      /* 1  LEDNODE_WIFI_CONN (not a flash) */
  0,                      /* 2  LEDNODE_IDLE      (not a flash) */
  0,                      /* 3  LEDNODE_STRESS    (not a flash) */
  LEDNODE_TICKS_HTTP,     /* 4  LEDNODE_HTTP_REQ  */
  LEDNODE_TICKS_INFER,    /* 5  LEDNODE_INFER     */
  LEDNODE_TICKS_PEER,     /* 6  LEDNODE_PEER_FOUND */
  LEDNODE_TICKS_HB_TX,    /* 7  LEDNODE_HB_TX     */
  LEDNODE_TICKS_HB_FAIL,  /* 8  LEDNODE_HB_FAIL   */
};

/****************************************************************************
 * Public Data (shared globals — declared extern in lednode.h)
 ****************************************************************************/

volatile int g_led_base  = LEDNODE_BOOT;
volatile int g_led_flash = -1;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int      s_fd         = -1;    /* /dev/rmt0 file descriptor */
static uint32_t s_rmt[RMT_TOTAL_WORDS];  /* encoded RMT frame buffer */

/* Animation counters */
static uint32_t s_tick       = 0;    /* global 50ms tick counter */
static int      s_flash_ev   = -1;   /* current active flash event */
static int      s_flash_left = 0;    /* ticks remaining for flash */
static int      s_flash_tick = 0;    /* ticks since flash started */

/* WiFi state tracking */
static uint32_t s_last_wifi_check = 0;
static int      s_had_ip          = 0; /* 1 after first IP detected */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: triwave
 * Description: Integer triangle wave 0→max→0 over `period` ticks.
 *              Produces a smooth breathing effect without floating point.
 ****************************************************************************/

static uint8_t triwave(uint32_t tick, uint32_t period, uint8_t max_val)
{
  uint32_t phase = tick % period;
  uint32_t half  = period / 2;
  if (half == 0) return 0;
  if (phase < half)
    return (uint8_t)((uint32_t)max_val * phase / half);
  else
    return (uint8_t)((uint32_t)max_val * (period - phase) / half);
}

/****************************************************************************
 * Name: ws2812_encode_byte
 * Description: Encode one byte (MSB first) into 8 RMT words at `dst`.
 ****************************************************************************/

static void ws2812_encode_byte(uint8_t byte, uint32_t *dst)
{
  int i;
  for (i = 7; i >= 0; i--)
    {
      if (byte & (1 << i))
        dst[7 - i] = ((uint32_t)T1L << 16) | (0x8000u | T1H);
      else
        dst[7 - i] = ((uint32_t)T0L << 16) | (0x8000u | T0H);
    }
}

/****************************************************************************
 * Name: led_write
 * Description: Encode (r,g,b) and send one WS2812 frame via /dev/rmt0.
 *              WS2812 wire order is GRB.
 ****************************************************************************/

static void led_write(uint8_t r, uint8_t g, uint8_t b)
{
  if (s_fd < 0) return;

  /* Scale by global brightness */
  r = (uint8_t)((uint32_t)r * LED_BRIGHTNESS / 255);
  g = (uint8_t)((uint32_t)g * LED_BRIGHTNESS / 255);
  b = (uint8_t)((uint32_t)b * LED_BRIGHTNESS / 255);

  ws2812_encode_byte(g, s_rmt);        /* GRB order: green first */
  ws2812_encode_byte(r, s_rmt + 8);
  ws2812_encode_byte(b, s_rmt + 16);
  s_rmt[RMT_DATA_WORDS] = 0;          /* reset/end-of-frame marker */

  write(s_fd, s_rmt, sizeof(s_rmt));
}

/****************************************************************************
 * Name: render_state
 * Description: Compute and emit RGB for the given state at the current tick.
 *              `t` is ticks since this state started (for events) or the
 *              global tick (for base states).
 ****************************************************************************/

static void render_state(int state, uint32_t t)
{
  uint8_t r = 0, g = 0, b = 0;

  switch (state)
    {
      /* ── Base states ─────────────────────────────────────────────────── */

      case LEDNODE_BOOT:
        /* White breathing, 3 s period (60 ticks) */
        {
          uint8_t v = triwave(t, 60, 255);
          r = v; g = v; b = v;
        }
        break;

      case LEDNODE_WIFI_CONN:
        /* Blue 1 Hz blink (20 ticks period, on for first 10) */
        if ((t % 20) < 10)
          b = 255;
        break;

      case LEDNODE_IDLE:
        /* Slow green breathing, 4 s period (80 ticks), faint cyan tint */
        {
          uint8_t v = 30 + triwave(t, 80, 225);
          g = v;
          b = v / 6;
        }
        break;

      case LEDNODE_STRESS:
        /* Orange solid — CPU is loaded */
        r = 255; g = 80; b = 0;
        break;

      /* ── Flash events ────────────────────────────────────────────────── */

      case LEDNODE_HTTP_REQ:
        /* White 100ms flash */
        r = 255; g = 255; b = 255;
        break;

      case LEDNODE_INFER:
        /* Purple 200ms flash */
        r = 180; g = 0; b = 255;
        break;

      case LEDNODE_PEER_FOUND:
        /* Cyan double-flash: on(2)-off(2)-on(2)-off(4) = 10 ticks */
        {
          int p = (int)(t % 10);
          if (p < 2 || (p >= 4 && p < 6))
            {
              g = 0; b = 255; r = 0;  /* cyan = green+blue */
              g = 255;
            }
        }
        break;

      case LEDNODE_HB_TX:
        /* Blue 50ms flash */
        b = 200;
        break;

      case LEDNODE_HB_FAIL:
        /* Red 300ms flash */
        r = 255;
        break;

      default:
        break;
    }

  led_write(r, g, b);
}

/****************************************************************************
 * Name: check_wifi
 * Description: Poll wlan0 IP address and update g_led_base accordingly.
 *              Called every WIFI_CHECK_TICKS to avoid socket overhead.
 ****************************************************************************/

static void check_wifi(void)
{
  /* Do not override user-set STRESS state */
  if (g_led_base == LEDNODE_STRESS) return;

  struct ifreq     ifr;
  int              fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return;

  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_name, "wlan0", IFNAMSIZ);

  if (ioctl(fd, SIOCGIFADDR, &ifr) == 0)
    {
      struct sockaddr_in *sin = (struct sockaddr_in *)&ifr.ifr_addr;
      if (sin->sin_addr.s_addr != 0)
        {
          if (!s_had_ip)
            {
              /* Newly connected — trigger a brief green triple flash */
              s_had_ip  = 1;
              s_tick    = 0;   /* reset tick so breathing starts from 0 */
            }
          g_led_base = LEDNODE_IDLE;
        }
      else
        {
          s_had_ip   = 0;
          g_led_base = LEDNODE_WIFI_CONN;
        }
    }
  else
    {
      /* Interface not up yet */
      s_had_ip   = 0;
      g_led_base = LEDNODE_WIFI_CONN;
    }

  close(fd);
}

/****************************************************************************
 * Name: process_flash_request
 * Description: Check g_led_flash and start/upgrade the current flash event.
 *              An incoming event replaces the current one only if its
 *              priority (LEDNODE_* numeric value) is ≥ running event.
 ****************************************************************************/

static void process_flash_request(void)
{
  int req = g_led_flash;
  if (req < LEDNODE_HTTP_REQ) return;         /* not a flash event */

  /* Accept if: no flash running, OR incoming priority ≥ current */
  if (s_flash_left <= 0 || req >= s_flash_ev)
    {
      s_flash_ev   = req;
      s_flash_left = k_flash_ticks[req];
      s_flash_tick = 0;
    }

  g_led_flash = -1;  /* acknowledge — caller writes again for next event */
}

/****************************************************************************
 * lednode_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  printf("[led] openvela LED node daemon  (WS2812 via %s)\n", RMT_DEV);
  printf("[led] tip: run as 'lednode &'\n");

  /* Open RMT device */
  s_fd = open(RMT_DEV, O_WRONLY);
  if (s_fd < 0)
    {
      fprintf(stderr, "[led] ERROR: open %s failed: %d (%s)\n",
              RMT_DEV, errno, strerror(errno));
      fprintf(stderr, "[led] Make sure CONFIG_ESP_RMT=y and "
              "CONFIG_RMTCHAR=y are set.\n");
      return 1;
    }

  printf("[led] /dev/rmt0 opened OK — starting animation loop\n");

  /* Initial WiFi check */
  check_wifi();

  /* ── Main animation loop ──────────────────────────────────────────── */
  for (;;)
    {
      /* 1. Check for new flash requests from other tasks */
      process_flash_request();

      /* 2. Periodic WiFi state detection */
      if (s_tick - s_last_wifi_check >= WIFI_CHECK_TICKS)
        {
          s_last_wifi_check = s_tick;
          check_wifi();
        }

      /* 3. Render: flash overrides base if active */
      if (s_flash_left > 0)
        {
          render_state(s_flash_ev, (uint32_t)s_flash_tick);
          s_flash_tick++;
          s_flash_left--;
        }
      else
        {
          render_state(g_led_base, s_tick);
        }

      /* 4. Advance global tick and sleep one frame */
      s_tick++;
      usleep(TICK_MS * 1000);
    }

  close(s_fd);
  return 0;
}
