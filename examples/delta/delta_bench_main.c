/****************************************************************************
 * examples/delta/delta_bench_main.c
 *
 * δ scheduling benchmark for openvela ESP32-S3.
 *
 * Models the 4-task IMU processing pipeline:
 *   recv      — receive IMU data frame (WiFi UDP or simulated)
 *   preprocess — timestamp alignment + downsampling
 *   infer      — TFLite Micro inference (or stub)
 *   tx         — transmit result via WiFi UDP to Pi4B
 *
 * Runs three scheduling strategies (B_seq / B_systemd / C2) using
 * the c_scheduler, then EXECUTES each schedule with real pthreads +
 * message queues, measures actual wall time, and reports speedup.
 *
 * Usage:  delta_bench [--dry]    (--dry = skip task execution, algo only)
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <semaphore.h>

#include "c_scheduler.h"

#ifdef CONFIG_EXAMPLES_DELTA_TFLITE
extern int tflm_infer_init(void);
extern int tflm_infer_once(float input, float *output);
#endif

/****************************************************************************
 * Task duration profile (milliseconds) — from S3 ostest / real measurement.
 * Update these after real profiling.
 ****************************************************************************/

/* Dual-IMU pipeline: Movella DOT x2 (left + right leg).
 * recv receives both IMU frames, then LEFT and RIGHT preprocessing
 * run IN PARALLEL, fuse merges them, infer runs inference, tx sends.
 *
 *   wake → recv → preprocess_L ─┐
 *                 preprocess_R ─┴→ fuse → infer → tx
 *
 * B_seq serializes all 6 tasks; C2 parallelizes preprocess_L/R.
 */
#define DUR_RECV        5.0f   /* UDP recv both IMU frames ~5 ms */
#define DUR_PREPROC_L  20.0f   /* left leg: timestamp align + quat ~20 ms */
#define DUR_PREPROC_R  20.0f   /* right leg: timestamp align + quat ~20 ms */
#define DUR_FUSE       10.0f   /* sensor fusion: relative angle ~10 ms */
#define DUR_INFER      35.0f   /* TFLite Micro stub / real ~30-50 ms */
#define DUR_TX          5.0f   /* UDP sendto result ~5 ms */

#define IPC_DELTA       0.5f   /* NuttX mq latency ~0.5 ms (measured) */
#define NODE_NAME      "s3"

/****************************************************************************
 * Semaphore-based execution engine — supports fan-out / fan-in DAGs.
 *
 * One semaphore per edge.  Each task:
 *   1. Waits until its scheduled start offset has elapsed.
 *   2. Waits (sem_wait) for every upstream semaphore.
 *   3. Does simulated work (nanosleep).
 *   4. Posts (sem_post) to every downstream semaphore.
 ****************************************************************************/

#define N_TASKS  6
#define N_EDGES  6
#define MAX_DEPS 3

/* One semaphore per edge:
 *   sem[0]: recv      → preproc_L
 *   sem[1]: recv      → preproc_R
 *   sem[2]: preproc_L → fuse
 *   sem[3]: preproc_R → fuse
 *   sem[4]: fuse      → infer
 *   sem[5]: infer     → tx
 */
static sem_t g_sems[N_EDGES];

typedef struct
{
  int wait_sem[MAX_DEPS + 1]; /* -1 terminated */
  int post_sem[MAX_DEPS + 1]; /* -1 terminated */
} task_sync_t;

/* Wiring table for the dual-IMU graph */
static const task_sync_t g_sync[N_TASKS] =
{
  /* recv      */ {{-1},        {0, 1, -1}},
  /* preproc_L */ {{0, -1},     {2, -1}   },
  /* preproc_R */ {{1, -1},     {3, -1}   },
  /* fuse      */ {{2, 3, -1},  {4, -1}   },
  /* infer     */ {{4, -1},     {5, -1}   },
  /* tx        */ {{5, -1},     {-1}      },
};

static const char *g_task_names[N_TASKS] =
{
  "recv", "preproc_L", "preproc_R", "fuse", "infer", "tx"
};

static float g_task_dur[N_TASKS] =
{
  DUR_RECV, DUR_PREPROC_L, DUR_PREPROC_R, DUR_FUSE, DUR_INFER, DUR_TX
};

typedef struct
{
  int         idx;           /* task index 0-5 */
  float       start_offset;  /* scheduled start from t0 (ms) */
} task_ctx_t;

static struct timespec g_t0;

static float elapsed_ms(void)
{
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return (float)(now.tv_sec  - g_t0.tv_sec)  * 1000.0f +
         (float)(now.tv_nsec - g_t0.tv_nsec) / 1e6f;
}

static void sleep_ms(float ms)
{
  if (ms <= 0.0f) return;
  struct timespec ts;
  ts.tv_sec  = (long)(ms / 1000.0f);
  ts.tv_nsec = (long)((ms - (float)ts.tv_sec * 1000.0f) * 1e6f);
  nanosleep(&ts, NULL);
}

static void *task_thread(void *arg)
{
  task_ctx_t        *ctx  = (task_ctx_t *)arg;
  int                idx  = ctx->idx;
  const task_sync_t *sync = &g_sync[idx];

  /* 1. Wait until scheduled start offset */
  float wait = ctx->start_offset - elapsed_ms();
  if (wait > 0.0f) sleep_ms(wait);

  /* 2. Wait for all upstream semaphores */
  for (int i = 0; sync->wait_sem[i] != -1; i++)
    {
      sem_wait(&g_sems[sync->wait_sem[i]]);
    }

  float actual_start = elapsed_ms();
  printf("[delta] %-12s  start=%6.1f ms\n", g_task_names[idx], actual_start);

  /* 3. Do work — real TFLite for infer task, usleep otherwise */
#ifdef CONFIG_EXAMPLES_DELTA_TFLITE
  if (idx == 4)  /* infer = index 4 */
    {
      float out = 0.0f;
      tflm_infer_once(0.5f + (float)ctx->start_offset * 0.01f, &out);
    }
  else
#endif
    {
      sleep_ms(g_task_dur[idx]);
    }

  float actual_ready = elapsed_ms();
  printf("[delta] %-12s  ready=%6.1f ms\n", g_task_names[idx], actual_ready);

  /* 4. Post to all downstream semaphores */
  for (int i = 0; sync->post_sem[i] != -1; i++)
    {
      sem_post(&g_sems[sync->post_sem[i]]);
    }

  return NULL;
}

static float execute_schedule(const cs_schedule_t *sched,
                               const cs_graph_t    *graph,
                               bool                 dry)
{
  if (dry) return sched->T;

  /* Initialize semaphores (all locked) */
  for (int i = 0; i < N_EDGES; i++)
    {
      sem_init(&g_sems[i], 0, 0);
    }

  /* Build task contexts: extract scheduled start times */
  task_ctx_t ctx[N_TASKS];
  for (int t = 0; t < N_TASKS; t++)
    {
      ctx[t].idx          = t;
      ctx[t].start_offset = 0.0f;
      for (int j = 0; j < sched->n_order; j++)
        {
          int vi = sched->order[j];
          if (vi < graph->n_services &&
              strcmp(graph->services[vi].sid, g_task_names[t]) == 0)
            {
              ctx[t].start_offset = sched->t_start[j];
              break;
            }
        }
    }

  /* Record t0 and spawn all threads */
  clock_gettime(CLOCK_MONOTONIC, &g_t0);

  pthread_t threads[N_TASKS];
  for (int t = 0; t < N_TASKS; t++)
    {
      pthread_attr_t attr;
      pthread_attr_init(&attr);

#ifdef CONFIG_EXAMPLES_DELTA_TFLITE
      /* TFLite infer task needs larger stack for C++ inference */
      if (t == 4)
        {
          pthread_attr_setstacksize(&attr, 16384);
        }
#endif

      pthread_create(&threads[t], &attr, task_thread, &ctx[t]);
      pthread_attr_destroy(&attr);
    }

  for (int t = 0; t < N_TASKS; t++)
    {
      pthread_join(threads[t], NULL);
    }

  float wall = elapsed_ms();

  for (int i = 0; i < N_EDGES; i++)
    {
      sem_destroy(&g_sems[i]);
    }

  return wall;
}

/****************************************************************************
 * Build the 4-task pipeline graph
 ****************************************************************************/

static void build_graph(cs_graph_t *g)
{
  cs_graph_init(g);

  cs_add_service(g, "recv",      NODE_NAME, DUR_RECV);
  cs_add_service(g, "preproc_L", NODE_NAME, DUR_PREPROC_L);
  cs_add_service(g, "preproc_R", NODE_NAME, DUR_PREPROC_R);
  cs_add_service(g, "fuse",      NODE_NAME, DUR_FUSE);
  cs_add_service(g, "infer",     NODE_NAME, DUR_INFER);
  cs_add_service(g, "tx",        NODE_NAME, DUR_TX);

  cs_add_edge(g, CS_WAKE_ID,  "recv",      0.0f);
  cs_add_edge(g, "recv",      "preproc_L", IPC_DELTA);
  cs_add_edge(g, "recv",      "preproc_R", IPC_DELTA);
  cs_add_edge(g, "preproc_L", "fuse",      IPC_DELTA);
  cs_add_edge(g, "preproc_R", "fuse",      IPC_DELTA);
  cs_add_edge(g, "fuse",      "infer",     IPC_DELTA);
  cs_add_edge(g, "infer",     "tx",        IPC_DELTA);

  const char *terms[] = {"tx"};
  cs_set_terminals(g, terms, 1);
}

/****************************************************************************
 * Main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  bool dry = false;

  for (int i = 1; i < argc; i++)
    {
      if (strcmp(argv[i], "--dry") == 0)
        {
          dry = true;
        }
    }

  printf("\n");
  printf("╔══════════════════════════════════════════╗\n");
  printf("║  δ Scheduling Benchmark  (openvela S3)   ║\n");
  printf("╚══════════════════════════════════════════╝\n");
  printf("  recv(%.0f) → [preproc_L(%.0f) || preproc_R(%.0f)]"
         " → fuse(%.0f) → infer(%.0f) → tx(%.0f) ms\n",
         DUR_RECV, DUR_PREPROC_L, DUR_PREPROC_R,
         DUR_FUSE, DUR_INFER, DUR_TX);
  printf("  IPC delta: %.1f ms   Mode: %s\n\n",
         IPC_DELTA, dry ? "DRY (algo only)" : "EXECUTE (real pthreads)");

  cs_graph_t    graph;
  cs_schedule_t sched_seq;
  cs_schedule_t sched_c2;

  build_graph(&graph);

#ifdef CONFIG_EXAMPLES_DELTA_TFLITE
  printf("  Initializing TFLite Micro interpreter...\n");
  if (tflm_infer_init() != 0)
    {
      printf("  ERROR: TFLite init failed\n");
      return EXIT_FAILURE;
    }

  printf("  TFLite ready.\n\n");
#endif

  /* Compute schedules */
  if (cs_schedule_b_seq(&graph, &sched_seq) != 0)
    {
      printf("[delta] ERROR: B_seq scheduling failed\n");
      return EXIT_FAILURE;
    }

  if (cs_schedule_c2(&graph, &sched_c2) != 0)
    {
      printf("[delta] ERROR: C2 scheduling failed\n");
      return EXIT_FAILURE;
    }

  cs_print_schedule(&sched_seq, &graph);
  cs_print_schedule(&sched_c2,  &graph);

  printf("\n--- Theoretical speedup: %.3f×\n",
         cs_speedup(&sched_seq, &sched_c2));

  /* Execute schedules and measure wall time */
  printf("\n--- Executing B_seq ---\n");
  float wall_seq = execute_schedule(&sched_seq, &graph, dry);

  printf("\n--- Executing C2 ---\n");
  float wall_c2  = execute_schedule(&sched_c2, &graph, dry);

  /* Results */
  printf("\n");
  printf("╔══════════════════════════════════════════╗\n");
  printf("║  RESULTS                                  ║\n");
  printf("╠══════════════════════════════════════════╣\n");
  printf("║  B_seq wall time : %6.1f ms              ║\n", wall_seq);
  printf("║  C2    wall time : %6.1f ms              ║\n", wall_c2);
  if (wall_c2 > 0.0f)
    {
      printf("║  Measured speedup: %6.3f×               ║\n",
             wall_seq / wall_c2);
    }

  printf("╚══════════════════════════════════════════╝\n\n");

  return EXIT_SUCCESS;
}
