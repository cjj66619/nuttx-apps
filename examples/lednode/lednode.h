/****************************************************************************
 * apps/examples/lednode/lednode.h
 *
 * Shared LED feedback API for openvela ESP32-S3 distributed AI node.
 * All tasks (httpnode, inferd, etc.) include this header to signal events.
 *
 * In NuttX flat/builtin mode all tasks share one address space, so volatile
 * globals defined in lednode_main.c are visible to every linked app.
 *
 * Usage:
 *   lednode_base(LEDNODE_IDLE);          // persistent state change
 *   lednode_flash(LEDNODE_HTTP_REQ);     // timed flash event
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_LEDNODE_LEDNODE_H
#define __APPS_EXAMPLES_LEDNODE_LEDNODE_H

/****************************************************************************
 * Base states (persistent until explicitly changed)
 ****************************************************************************/

#define LEDNODE_BOOT         0   /* startup: white breathing  */
#define LEDNODE_WIFI_CONN    1   /* WiFi connecting: blue 1Hz blink */
#define LEDNODE_IDLE         2   /* all good: slow green breathing */
#define LEDNODE_STRESS       3   /* inferd stress: orange solid */

/****************************************************************************
 * Flash events (auto-expire, override base for fixed duration)
 *
 * Higher value = higher priority (HB_FAIL wins over HTTP_REQ).
 ****************************************************************************/

#define LEDNODE_HTTP_REQ     4   /* browser request: white 100ms */
#define LEDNODE_INFER        5   /* inference done: purple 200ms */
#define LEDNODE_PEER_FOUND   6   /* peer discovered: cyan double-flash */
#define LEDNODE_HB_TX        7   /* heartbeat TX: blue 60ms */
#define LEDNODE_HB_FAIL      8   /* heartbeat fail: red 300ms */

/****************************************************************************
 * Flash event durations in 50ms ticks
 ****************************************************************************/

#define LEDNODE_TICKS_HTTP    2   /*  100ms */
#define LEDNODE_TICKS_INFER   4   /*  200ms */
#define LEDNODE_TICKS_PEER   10   /*  500ms (double-flash pattern) */
#define LEDNODE_TICKS_HB_TX   1   /*   50ms */
#define LEDNODE_TICKS_HB_FAIL 6   /*  300ms */

/****************************************************************************
 * Shared globals  — defined in lednode_main.c, extern everywhere else.
 *
 * When CONFIG_EXAMPLES_LEDNODE is not set the macros become no-ops so
 * httpnode / inferd compile cleanly without the LED daemon.
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_LEDNODE

extern volatile int g_led_base;    /* current base state */
extern volatile int g_led_flash;   /* -1 = none; else LEDNODE_* event */

/* Set persistent base state (does NOT clear an in-progress flash) */
#define lednode_base(s)   do { g_led_base  = (s); } while (0)

/* Request a timed flash event (lednode daemon picks it up on next tick) */
#define lednode_flash(e)  do { g_led_flash = (e); } while (0)

#else  /* !CONFIG_EXAMPLES_LEDNODE */

#define lednode_base(s)   do { } while (0)
#define lednode_flash(e)  do { } while (0)

#endif /* CONFIG_EXAMPLES_LEDNODE */

#endif /* __APPS_EXAMPLES_LEDNODE_LEDNODE_H */
