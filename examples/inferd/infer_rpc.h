/****************************************************************************
 * apps/examples/inferd/infer_rpc.h
 *
 * Shared types for distributed inference and δ-C2 adaptive scheduling.
 *
 * Wire protocol  (port INFERD_PORT = 4446, little-endian):
 *   request  : 4 bytes  — float32 input
 *   response : 8 bytes  — float32 result  +  uint32_t infer_us
 *
 * δ-C2 peer table  (globals defined in inferd_main.c):
 *   dc2_update_peer()  — called by nodelink_daemon when a heartbeat
 *                        with a "cpu":xx.x field is received.
 *   dc2_decide()       — called by inferd daemon to choose local vs
 *                        offload execution path.
 ****************************************************************************/

#pragma once

#include <stdint.h>

/****************************************************************************
 * Protocol constants
 ****************************************************************************/

#define INFERD_PORT           4446    /* TCP port for inference worker      */

/****************************************************************************
 * δ-C2 thresholds (tunable)
 ****************************************************************************/

#define DC2_MAX_PEERS         4       /* max tracked peer nodes             */
#define DC2_LOCAL_CPU_THRESH  65.0f   /* offload when local CPU% > this     */
#define DC2_OFFLOAD_TO_MS     2000    /* TCP connect+recv timeout (ms)      */

/****************************************************************************
 * Peer state  (one entry per discovered openvela node)
 ****************************************************************************/

typedef struct
{
  char  ip[16];        /* null-terminated dotted-decimal, e.g. "192.168.1.2" */
  float cpu_pct;       /* 0-100 last known CPU load; negative = unknown       */
} dc2_peer_t;

extern dc2_peer_t     g_dc2_peers[DC2_MAX_PEERS];
extern volatile int   g_dc2_n_peers;

/****************************************************************************
 * δ-C2 decision type
 ****************************************************************************/

typedef enum
{
  DC2_LOCAL   = 0,
  DC2_OFFLOAD = 1,
} dc2_decision_t;

/****************************************************************************
 * API
 ****************************************************************************/

/* Choose execution path.
 * Returns DC2_OFFLOAD and sets *peer_ip_out when a suitable peer exists.
 * Returns DC2_LOCAL otherwise (*peer_ip_out is left unchanged). */

dc2_decision_t dc2_decide(float local_cpu_pct, const char **peer_ip_out);

/* Register or refresh a peer's CPU load.  Safe to call from any task
 * (nodelink_daemon calls this when it receives a heartbeat). */

void dc2_update_peer(const char *ip, float cpu_pct);
