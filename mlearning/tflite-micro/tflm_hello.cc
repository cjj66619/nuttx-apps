/****************************************************************************
 * apps/mlearning/tflite-micro/tflm_hello.cc
 *
 * TFLite Micro feasibility gate for openvela ESP32-S3.
 * Runs hello_world_int8 model (sine approximation), measures inference
 * latency, and reports result. No filesystem required — model is embedded.
 *
 * Usage: tflm_hello
 ****************************************************************************/

#include <nuttx/config.h>

#include <cstdio>
#include <cmath>
#include <ctime>

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

/* Embedded model data */
extern const unsigned char g_hello_world_model[];
extern const unsigned int  g_hello_world_model_len;

/* Tensor arena size — 10KB is enough for hello_world */
constexpr int kTensorArenaSize = 10 * 1024;
static uint8_t tensor_arena[kTensorArenaSize] __attribute__((aligned(16)));

/* Singleton interpreter for pipeline use */
static tflite::MicroInterpreter *g_interpreter = nullptr;

/**
 * tflm_infer_once - C-callable inference entry point for delta pipeline.
 * Initializes the interpreter on first call (lazy init).
 * @input:   float input value (will be quantized to int8)
 * @output:  float output (dequantized from int8)
 * @returns: 0 on success, -1 on error
 */
/* Global resolver — avoids magic-static init in pthreads */
static tflite::MicroMutableOpResolver<4> g_resolver;
static bool g_tflm_ready = false;

/* Storage for interpreter — placement new avoids heap */
static uint8_t g_interp_buf[sizeof(tflite::MicroInterpreter)]
               __attribute__((aligned(16)));

extern "C" int tflm_infer_init(void)
{
  if (g_tflm_ready) return 0;

  const tflite::Model *model = tflite::GetModel(g_hello_world_model);
  if (model->version() != TFLITE_SCHEMA_VERSION) return -1;

  g_resolver.AddFullyConnected();
  g_resolver.AddQuantize();
  g_resolver.AddDequantize();
  g_resolver.AddReshape();

  g_interpreter = new(g_interp_buf) tflite::MicroInterpreter(
    model, g_resolver, tensor_arena, kTensorArenaSize);

  if (g_interpreter->AllocateTensors() != kTfLiteOk)
    {
      g_interpreter = nullptr;
      return -1;
    }

  g_tflm_ready = true;
  return 0;
}

extern "C" int tflm_infer_once(float input, float *output)
{
  if (!g_tflm_ready || !g_interpreter) return -1;

  TfLiteTensor *in  = g_interpreter->input(0);
  TfLiteTensor *out = g_interpreter->output(0);

  in->data.int8[0] = (int8_t)(input / in->params.scale
                               + in->params.zero_point);

  if (g_interpreter->Invoke() != kTfLiteOk) return -1;

  if (output)
    {
      *output = (out->data.int8[0] - out->params.zero_point)
                * out->params.scale;
    }

  return 0;
}

static float elapsed_ms_since(const struct timespec &t0)
{
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return (float)(now.tv_sec  - t0.tv_sec)  * 1000.0f +
         (float)(now.tv_nsec - t0.tv_nsec) / 1e6f;
}

extern "C" int tflm_hello_main(int argc, char *argv[])
{
  printf("\n╔══════════════════════════════════════════╗\n");
  printf("║  TFLite Micro — hello_world  (S3)        ║\n");
  printf("╚══════════════════════════════════════════╝\n");
  printf("  Model: hello_world_int8 (%u bytes)\n", g_hello_world_model_len);

  /* Load model */
  const tflite::Model *model =
    tflite::GetModel(g_hello_world_model);

  if (model->version() != TFLITE_SCHEMA_VERSION)
    {
      printf("  ERROR: schema mismatch (%d vs %d)\n",
             (int)model->version(), TFLITE_SCHEMA_VERSION);
      return 1;
    }

  /* Register only the ops needed by hello_world (fully_connected + quantize) */
  tflite::MicroMutableOpResolver<4> resolver;
  resolver.AddFullyConnected();
  resolver.AddQuantize();
  resolver.AddDequantize();
  resolver.AddReshape();

  /* Build interpreter */
  tflite::MicroInterpreter interpreter(
    model, resolver, tensor_arena, kTensorArenaSize);

  if (interpreter.AllocateTensors() != kTfLiteOk)
    {
      printf("  ERROR: AllocateTensors failed\n");
      return 1;
    }

  TfLiteTensor *input  = interpreter.input(0);
  TfLiteTensor *output = interpreter.output(0);

  printf("  Input  shape: [%d] type=%d\n",
         (int)input->dims->data[0], (int)input->type);
  printf("  Output shape: [%d] type=%d\n",
         (int)output->dims->data[0], (int)output->type);
  printf("  Tensor arena used: %u / %d bytes\n\n",
         (unsigned)interpreter.arena_used_bytes(), kTensorArenaSize);

  /* Run inference at 8 points: x = 0, π/4, π/2, ..., 7π/4 */
  printf("  %-10s  %-12s  %-12s  %-12s  %s\n",
         "x(rad)", "expected(sin)", "inferred", "error", "latency");
  printf("  %s\n", "--------------------------------------------------------------");

  float total_ms = 0.0f;

  for (int i = 0; i < 8; i++)
    {
      float x = (float)i * M_PI / 4.0f;
      float expected = sinf(x);

      /* Quantize input: int8 = x / input_scale + input_zero_point */
      int8_t input_q = (int8_t)(x / input->params.scale +
                                input->params.zero_point);
      input->data.int8[0] = input_q;

      struct timespec t0;
      clock_gettime(CLOCK_MONOTONIC, &t0);

      if (interpreter.Invoke() != kTfLiteOk)
        {
          printf("  ERROR: Invoke failed at i=%d\n", i);
          return 1;
        }

      float latency = elapsed_ms_since(t0);
      total_ms += latency;

      /* Dequantize output */
      float inferred = (output->data.int8[0] - output->params.zero_point)
                       * output->params.scale;

      printf("  %-10.4f  %-12.4f  %-12.4f  %-12.4f  %.2f ms\n",
             x, expected, inferred, fabsf(expected - inferred), latency);
    }

  printf("\n  Average inference latency: %.2f ms\n", total_ms / 8.0f);
  printf("\n  ✅ TFLite Micro feasibility gate: PASS\n\n");

  return 0;
}
