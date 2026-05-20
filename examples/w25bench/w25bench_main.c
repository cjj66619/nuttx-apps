/****************************************************************************
 * apps/examples/w25bench/w25bench_main.c
 *
 * W25Q NOR Flash micro-benchmark.
 *
 *   This builtin measures read / write throughput against /dev/w25 through
 *   the standard VFS path (the kernel's auto block_proxy() wraps the MTD
 *   inode with FTL + BCH on first open).  The numbers therefore *include*
 *   the FTL + BCH stack overhead -- which is intentional: the W2 D3 baseline
 *   has to compare against the W2 D6 DMA build using the *same* path,
 *   otherwise the speed-up would be overstated.
 *
 *   The tool prints one human-readable summary line per (op,size) and one
 *   CSV-style line that can be pasted directly into docs/perf/w25-*.md.
 *
 * Usage:
 *
 *   w25bench --op=<read|write> --size=<4k|64k|1m>
 *            [--iters=<N>]      default 5
 *            [--offset=<N>]     default 0 (byte offset on the chip)
 *            [--seed=<N>]       default 0xdeadbeef
 *            [--confirm-destructive]  required for --op=write
 *
 * Notes:
 *   - "write" is destructive: it overwrites NOR at <offset> .. <offset+size>.
 *     If LittleFS is mounted on /data and the test region overlaps the
 *     file-system area, *unmount /data first* (`umount /data`) -- otherwise
 *     the FS metadata will be corrupted.  After the test re-`mount` it; the
 *     autoformat in board bring-up will rebuild the superblock if needed.
 *   - "read" is non-destructive.
 *   - In W2 D3 only the POLL SPI mode is exercised (no Kconfig switch yet
 *     for DMA).  The --mode= switch is reserved for v2.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define W25BENCH_DEV       "/dev/w25"
#define DEFAULT_ITERS      5
#define DEFAULT_SEED       0xdeadbeefUL
#define MAX_ITERS          64
#define SEGMENT_BYTES      (64 * 1024)  /* 64 KiB heap segment for 1 MiB op */
#define W25_FLASH_BYTES    (16 * 1024 * 1024) /* NM25Q128 = 16 MiB */

/* Timer-precision floor.  CLOCK_MONOTONIC in this NuttX build is driven by
 * the 100 Hz systick -> only 10 ms resolution.  We pick 256 KiB so that
 * even at the W2 D3.1 baseline (~0.85 MB/s) one timed window spans roughly
 * 200 ms = 20 ticks -> < 5% quantization error.  Faster builds (SCK 42 MHz
 * / DMA) overshoot proportionally, but the floor still keeps every window
 * >> 1 tick.  For size >= MIN_TIMED_BYTES inner_loops collapses to 1 so the
 * v1 baseline behaviour is preserved byte-for-byte.
 */

#define MIN_TIMED_BYTES    (256 * 1024)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum op_e
{
  OP_READ = 0,
  OP_WRITE,
};

struct bench_cfg_s
{
  enum op_e op;
  size_t    size;        /* Total bytes per iteration */
  int       iters;
  off_t     offset;
  uint32_t  seed;
  bool      destructive_ok;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Print short usage hint and return 1 (so main can `return usage(...);`). */

static int usage(FAR const char *prog)
{
  fprintf(stderr,
          "Usage: %s --op=<read|write> --size=<4k|64k|1m>\n"
          "           [--iters=<N>] [--offset=<N>] [--seed=<N>]\n"
          "           [--confirm-destructive]\n"
          "\n"
          "Example: %s --op=read --size=64k --iters=5\n",
          prog, prog);
  return 1;
}

/* Translate --size=<token> to byte count.  Accepts the three tokens that
 * appear in the W2 D3 baseline matrix; anything else fails fast.
 */

static int parse_size(FAR const char *tok, FAR size_t *out)
{
  if (strcmp(tok, "4k")  == 0) { *out = 4   * 1024; return 0; }
  if (strcmp(tok, "64k") == 0) { *out = 64  * 1024; return 0; }
  if (strcmp(tok, "1m")  == 0) { *out = 1024 * 1024; return 0; }
  return -EINVAL;
}

/* Fill `buf` with a deterministic pseudo-random pattern.  Using rand_r so
 * that --seed reproduces the exact byte sequence across runs, which is
 * important for write-then-read verification later.
 */

static void fill_pattern(FAR uint8_t *buf, size_t n, uint32_t seed)
{
  uint32_t state = seed ? seed : 1;
  for (size_t i = 0; i < n; i++)
    {
      /* xorshift32 -- not crypto, just a cheap deterministic stream */

      state ^= state << 13;
      state ^= state >> 17;
      state ^= state << 5;
      buf[i] = (uint8_t)(state & 0xff);
    }
}

/* Return the elapsed nanoseconds between two CLOCK_MONOTONIC samples. */

static uint64_t ts_diff_ns(FAR const struct timespec *a,
                           FAR const struct timespec *b)
{
  return (uint64_t)(b->tv_sec - a->tv_sec) * 1000000000ULL +
         (uint64_t)(b->tv_nsec - a->tv_nsec);
}

/* Walk a sorted ns[] array and return its median value. */

static uint64_t median_u64(FAR uint64_t *arr, int n)
{
  /* Trivial insertion sort -- n <= MAX_ITERS so O(n^2) is fine. */

  for (int i = 1; i < n; i++)
    {
      uint64_t key = arr[i];
      int j = i - 1;
      while (j >= 0 && arr[j] > key)
        {
          arr[j + 1] = arr[j];
          j--;
        }
      arr[j + 1] = key;
    }

  return (n & 1) ? arr[n / 2] : ((arr[n / 2 - 1] + arr[n / 2]) / 2);
}

/* Loop read() / write() until all `len` bytes are transferred or the fd
 * starts returning short / failed results.  This is needed because the
 * BCH char proxy may chunk transfers internally.
 */

static ssize_t full_read(int fd, FAR void *buf, size_t len)
{
  size_t done = 0;
  FAR uint8_t *p = (FAR uint8_t *)buf;
  while (done < len)
    {
      ssize_t n = read(fd, p + done, len - done);
      if (n == 0)
        {
          return done;
        }
      if (n < 0)
        {
          if (errno == EINTR) continue;
          return -1;
        }
      done += (size_t)n;
    }

  return (ssize_t)done;
}

static ssize_t full_write(int fd, FAR const void *buf, size_t len)
{
  size_t done = 0;
  FAR const uint8_t *p = (FAR const uint8_t *)buf;
  while (done < len)
    {
      ssize_t n = write(fd, p + done, len - done);
      if (n == 0)
        {
          return done;
        }
      if (n < 0)
        {
          if (errno == EINTR) continue;
          return -1;
        }
      done += (size_t)n;
    }

  return (ssize_t)done;
}

/* Compute KB/s with two-decimal precision (kept in fixed point to avoid
 * pulling printf-float linker bloat unnecessarily, but stdio already has
 * float here so we just use %.2f).
 */

static double bytes_per_ns_to_kbps(size_t bytes, uint64_t ns)
{
  if (ns == 0) return 0.0;
  return ((double)bytes * 1000000.0) / ((double)ns * 1.024);
  /* bytes / ns          = GB/s
   * bytes / ns * 1e6    = MB/s? no -> derive carefully:
   *   bytes/s   = bytes / (ns * 1e-9) = bytes * 1e9 / ns
   *   KB/s      = bytes * 1e9 / ns / 1024
   *             = (bytes * 1e6 / ns) * (1000 / 1024)
   *             ~= (bytes * 1e6 / ns) / 1.024
   * So we use bytes * 1e6 / ns / 1.024 -- the constant 1000000 / 1.024
   * yields KB/s (KiB/s actually, base-1024).
   */
}

/* Run the (op, size) iteration loop with a streaming buffer.
 *
 *   cfg->size is the *logical* per-iteration transfer (4 KiB / 64 KiB /
 *   1 MiB).  `buf` holds at most buf_cap bytes -- when the logical size is
 *   larger we loop, refilling buf as needed, but the wall-clock measurement
 *   spans the whole logical transfer.  This keeps the 1 MiB case viable on
 *   a 112 KiB-heap target.
 *
 *   The buffer pattern is regenerated *outside* the timed window for each
 *   write iteration so that the measurement reflects flash throughput, not
 *   xorshift latency.
 */

/* Per-iteration walker: do `inner_loops` of (lseek + transfer of size
 * bytes) under a single wall-clock window.  Returns 0 on success.
 *
 *   - For READ: every inner pass lseeks back to cfg->offset and reads
 *     size bytes -> same flash region every time, no side-effect.
 *   - For WRITE: every inner pass lseeks to (cfg->offset + j*size) and
 *     writes a fresh xorshift-derived pattern.  Wraps at W25_FLASH_BYTES
 *     so multi-iteration runs cannot escape the chip.
 *
 *   `effective_bytes` returns the total bytes that were transferred in
 *   the timed window (= size * inner_loops); the caller divides it by
 *   the elapsed ns to derive KB/s.
 */

static int run_one_window(FAR const char *prog, int fd,
                          FAR uint8_t *buf, size_t buf_cap,
                          FAR const struct bench_cfg_s *cfg,
                          int outer_idx, int inner_loops,
                          FAR uint64_t *ns_out,
                          FAR size_t *effective_bytes_out)
{
  struct timespec t0;
  struct timespec t1;

  clock_gettime(CLOCK_MONOTONIC, &t0);

  for (int j = 0; j < inner_loops; j++)
    {
      off_t this_offset;

      if (cfg->op == OP_READ)
        {
          this_offset = cfg->offset;
        }
      else
        {
          /* Slide the write window through the chip so we don't keep
           * hammering the same sector (which on NOR turns into a NOP
           * after one write without erase, masking real throughput).
           */

          uint64_t step = (uint64_t)(outer_idx * inner_loops + j) *
                          (uint64_t)cfg->size;
          uint64_t bound = (uint64_t)W25_FLASH_BYTES -
                           (uint64_t)cfg->offset;
          this_offset = cfg->offset + (off_t)(step % bound);
        }

      if (lseek(fd, this_offset, SEEK_SET) < 0)
        {
          fprintf(stderr, "[%s] lseek to %ld failed: %d\n",
                  prog, (long)this_offset, errno);
          return 1;
        }

      size_t remaining = cfg->size;
      while (remaining > 0)
        {
          size_t chunk = (remaining > buf_cap) ? buf_cap : remaining;
          ssize_t rc;

          if (cfg->op == OP_READ)
            {
              rc = full_read(fd, buf, chunk);
            }
          else
            {
              rc = full_write(fd, buf, chunk);
            }

          if (rc != (ssize_t)chunk)
            {
              fprintf(stderr,
                      "[%s] iter %d.%d: short xfer (%ld of %zu @ %zu rem, "
                      "errno=%d)\n",
                      prog, outer_idx, j, (long)rc, chunk, remaining,
                      errno);
              return 1;
            }

          remaining -= chunk;
        }
    }

  clock_gettime(CLOCK_MONOTONIC, &t1);

  *ns_out = ts_diff_ns(&t0, &t1);
  *effective_bytes_out = cfg->size * (size_t)inner_loops;
  return 0;
}

static int run_loop(FAR const char *prog, int fd, FAR uint8_t *buf,
                    size_t buf_cap,
                    FAR const struct bench_cfg_s *cfg,
                    FAR const char *size_tok)
{
  uint64_t ns_per_iter[MAX_ITERS];
  uint64_t ns_min = UINT64_MAX;
  uint64_t ns_max = 0;
  uint64_t ns_sum = 0;
  size_t effective_bytes = 0;

  /* Compute inner_loops so each timed window covers at least
   * MIN_TIMED_BYTES.  For size >= MIN_TIMED_BYTES this collapses to 1
   * (== v1 behaviour, no change for 1 MiB).
   */

  int inner_loops = (int)((MIN_TIMED_BYTES + cfg->size - 1) / cfg->size);
  if (inner_loops < 1)
    {
      inner_loops = 1;
    }

  for (int i = 0; i < cfg->iters; i++)
    {
      /* Pre-fill the buffer (writes only) outside the timed window so
       * xorshift latency doesn't pollute the measurement.  For writes
       * with inner_loops > 1 we re-seed per outer iter; consecutive
       * inner passes share the same buffer (their offsets differ, so
       * the bytes-on-flash still vary).
       */

      if (cfg->op == OP_WRITE)
        {
          fill_pattern(buf, buf_cap, cfg->seed ^ (uint32_t)i);
        }

      uint64_t ns = 0;
      size_t eff = 0;
      int rc = run_one_window(prog, fd, buf, buf_cap, cfg, i,
                              inner_loops, &ns, &eff);
      if (rc != 0)
        {
          return rc;
        }

      ns_per_iter[i] = ns;
      if (ns < ns_min) ns_min = ns;
      if (ns > ns_max) ns_max = ns;
      ns_sum += ns;
      effective_bytes = eff;  /* same value every iter */
    }

  uint64_t ns_med = median_u64(ns_per_iter, cfg->iters);

  /* KB/s is derived from the *effective* bytes per window, not from
   * cfg->size, so inner_loops >= 1 falls out cleanly.
   */

  double kbps_med  = bytes_per_ns_to_kbps(effective_bytes, ns_med);
  double kbps_best = bytes_per_ns_to_kbps(effective_bytes, ns_min);

  /* Human-readable line: scan-friendly, fits 80 columns.  We expose
   * inner_loops so the operator can see when the tool collapsed many
   * small transfers into one timed window.
   */

  printf("[%s] poll %-5s size=%-3s iters=%d inner=%-2d  "
         "med=%6llu us  rng=[%6llu..%6llu] us  "
         "KB/s med=%7.2f best=%7.2f\n",
         prog,
         cfg->op == OP_READ ? "read" : "write",
         size_tok,
         cfg->iters,
         inner_loops,
         (unsigned long long)(ns_med / 1000),
         (unsigned long long)(ns_min / 1000),
         (unsigned long long)(ns_max / 1000),
         kbps_med,
         kbps_best);

  /* Machine-friendly CSV (v2 schema): tag, mode, op, size_token,
   * size_bytes, inner_loops, iters, ns_min, ns_med, ns_max, ns_avg,
   * KB/s_med, KB/s_best.  v2 inserts inner_loops between size_bytes
   * and iters so older v1 lines and v2 lines are distinguishable by
   * column count (v1=12, v2=13).
   */

  printf("W25BENCH_CSV,poll,%s,%s,%zu,%d,%d,%llu,%llu,%llu,%llu,%.2f,%.2f\n",
         cfg->op == OP_READ ? "read" : "write",
         size_tok, cfg->size, inner_loops, cfg->iters,
         (unsigned long long)ns_min,
         (unsigned long long)ns_med,
         (unsigned long long)ns_max,
         (unsigned long long)(ns_sum / cfg->iters),
         kbps_med, kbps_best);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  struct bench_cfg_s cfg =
    {
      .op             = OP_READ,
      .size           = 0,
      .iters          = DEFAULT_ITERS,
      .offset         = 0,
      .seed           = DEFAULT_SEED,
      .destructive_ok = false,
    };

  bool op_set   = false;
  bool size_set = false;
  FAR const char *size_tok = "?";
  FAR const char *prog = argv[0] ? argv[0] : "w25bench";

  for (int i = 1; i < argc; i++)
    {
      FAR const char *a = argv[i];

      if      (strncmp(a, "--op=", 5) == 0)
        {
          FAR const char *v = a + 5;
          if      (strcmp(v, "read")  == 0) cfg.op = OP_READ;
          else if (strcmp(v, "write") == 0) cfg.op = OP_WRITE;
          else { fprintf(stderr, "bad --op: %s\n", v); return usage(prog); }
          op_set = true;
        }
      else if (strncmp(a, "--size=", 7) == 0)
        {
          size_tok = a + 7;
          if (parse_size(size_tok, &cfg.size) < 0)
            { fprintf(stderr, "bad --size: %s\n", size_tok); return usage(prog); }
          size_set = true;
        }
      else if (strncmp(a, "--iters=", 8) == 0)
        {
          cfg.iters = atoi(a + 8);
          if (cfg.iters < 1 || cfg.iters > MAX_ITERS)
            { fprintf(stderr, "iters out of range 1..%d\n", MAX_ITERS); return usage(prog); }
        }
      else if (strncmp(a, "--offset=", 9) == 0)
        {
          cfg.offset = (off_t)strtoul(a + 9, NULL, 0);
        }
      else if (strncmp(a, "--seed=", 7) == 0)
        {
          cfg.seed = (uint32_t)strtoul(a + 7, NULL, 0);
        }
      else if (strcmp(a, "--confirm-destructive") == 0)
        {
          cfg.destructive_ok = true;
        }
      else
        {
          fprintf(stderr, "unknown arg: %s\n", a);
          return usage(prog);
        }
    }

  if (!op_set || !size_set)
    {
      return usage(prog);
    }

  if (cfg.op == OP_WRITE && !cfg.destructive_ok)
    {
      fprintf(stderr,
              "[%s] --op=write rewrites flash at offset=%ld len=%zu.\n"
              "      Pass --confirm-destructive to acknowledge.\n"
              "      Also umount /data first if it overlaps this region.\n",
              prog, (long)cfg.offset, cfg.size);
      return 1;
    }

  /* Allocate a streaming buffer.  Cap at SEGMENT_BYTES (64 KiB) so that
   * even with --size=1m on a ~100 KiB free heap the allocation succeeds;
   * run_loop() then walks the logical 1 MiB transfer in 16 buffer-sized
   * chunks while the wall-clock spans the whole 1 MiB.
   */

  size_t buf_cap = (cfg.size > SEGMENT_BYTES) ? SEGMENT_BYTES : cfg.size;
  FAR uint8_t *buf = (FAR uint8_t *)malloc(buf_cap);
  if (buf == NULL)
    {
      fprintf(stderr,
              "[%s] malloc(%zu) failed -- not even %zu KiB free.\n",
              prog, buf_cap, buf_cap / 1024);
      return 1;
    }

  /* Poison the buffer for read tests: if the read silently returns 0 the
   * caller will at least see the poison pattern rather than stale 0xFF.
   */

  fill_pattern(buf, buf_cap, cfg.seed ^ 0x5a5a5a5aUL);

  int oflags = (cfg.op == OP_READ) ? O_RDONLY : O_RDWR;
  int fd = open(W25BENCH_DEV, oflags);
  if (fd < 0)
    {
      fprintf(stderr,
              "[%s] open(%s) failed: errno=%d.\n"
              "      Most common cause: /data is still mounted and the FS\n"
              "      driver is holding a private reference -- try `umount /data`.\n"
              "      Also check `ls /dev` shows `w25` (W2 D1 link).\n",
              prog, W25BENCH_DEV, errno);
      free(buf);
      return 1;
    }

  /* Banner: mirrors the columns the human-readable line below uses so the
   * serial capture is self-documenting.
   */

  printf("[%s] dev=%s op=%s size=%s(%zu) iters=%d offset=%ld seed=0x%08lx\n",
         prog, W25BENCH_DEV,
         cfg.op == OP_READ ? "read" : "write",
         size_tok, cfg.size, cfg.iters,
         (long)cfg.offset, (unsigned long)cfg.seed);

  int rc = run_loop(prog, fd, buf, buf_cap, &cfg, size_tok);

  close(fd);
  free(buf);
  return rc;
}
