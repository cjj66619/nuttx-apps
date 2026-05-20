/****************************************************************************
 * examples/delta/c_scheduler.h
 *
 * C port of the C2 dependency-graph-driven scheduler (Python spec:
 * c2/c2_scheduler.py, formalized in docs/c1-c2-formalization.md).
 *
 * Three strategies:
 *   SCHED_B_SEQ     — sequential baseline (§2.1)
 *   SCHED_B_SYSTEMD — per-node critical path, sequential inter-node (§2.2)
 *   SCHED_C2        — global dep-graph critical path (§3)
 *
 * Constraints: max CS_MAX_SERVICES services, CS_MAX_EDGES edges.
 * All timing in milliseconds (float).
 ****************************************************************************/

#ifndef __EXAMPLES_DELTA_C_SCHEDULER_H
#define __EXAMPLES_DELTA_C_SCHEDULER_H

#include <stddef.h>
#include <stdbool.h>

/****************************************************************************
 * Configuration
 ****************************************************************************/

#define CS_MAX_SERVICES  16
#define CS_MAX_EDGES     32
#define CS_MAX_NODES     8
#define CS_WAKE_ID       "_wake_"
#define CS_INF           1e9f

/****************************************************************************
 * Data structures (§1.1 – §1.3)
 ****************************************************************************/

typedef enum
{
  EDGE_LOCAL  = 0,
  EDGE_REMOTE = 1,
  EDGE_WAKE   = 2,
} edge_kind_t;

typedef struct
{
  char  sid[32];      /* unique service id */
  char  node[16];     /* hosting node h(s) */
  float duration;     /* d(s) in ms */
} cs_service_t;

typedef struct
{
  char        src[32];
  char        dst[32];
  float       delta;       /* latency in ms */
  edge_kind_t kind;
} cs_edge_t;

typedef struct
{
  cs_service_t services[CS_MAX_SERVICES];
  int          n_services;
  cs_edge_t    edges[CS_MAX_EDGES];
  int          n_edges;
  char         terminals[CS_MAX_SERVICES][32];
  int          n_terminals;
} cs_graph_t;

typedef struct
{
  const char  *strategy;
  float        t_start[CS_MAX_SERVICES + 1];  /* +1 for _wake_ */
  float        t_ready[CS_MAX_SERVICES + 1];
  float        T;                              /* end-to-end latency */
  int          order[CS_MAX_SERVICES + 1];     /* topo order indices */
  int          n_order;
} cs_schedule_t;

/****************************************************************************
 * Public API
 ****************************************************************************/

void cs_graph_init(cs_graph_t *g);

int  cs_add_service(cs_graph_t *g, const char *sid,
                    const char *node, float duration);

int  cs_add_edge(cs_graph_t *g, const char *src, const char *dst,
                 float delta);

void cs_set_terminals(cs_graph_t *g, const char **terminals, int n);

/* Schedule functions — return 0 on success, -1 on error */
int  cs_schedule_c2(cs_graph_t *g, cs_schedule_t *out);
int  cs_schedule_b_seq(cs_graph_t *g, cs_schedule_t *out);
int  cs_schedule_b_systemd(cs_graph_t *g, cs_schedule_t *out);

/* Utilities */
void cs_print_schedule(const cs_schedule_t *s, const cs_graph_t *g);
float cs_speedup(const cs_schedule_t *base, const cs_schedule_t *opt);

#endif /* __EXAMPLES_DELTA_C_SCHEDULER_H */
