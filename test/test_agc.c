/**
 * @file test_agc.c
 * @brief Native tests for the AGC library.
 *
 * Build and run:   make test
 *
 * The point of these is that they MEASURE rather than assume. The release is
 * specified as a time constant and implemented as a coefficient folded over a
 * whole block, so nothing here trusts the fold: the gain trajectory is
 * compared against a reference that runs the per-sample recursion literally,
 * one step at a time, and the time constant is recovered from the trajectory
 * and checked against the 50 ms that was asked for. That is the assertion that
 * catches the easiest mistake in this module by a factor of the block size.
 *
 * The rest is the shape of the loop rather than its speed: that a reduction
 * lands inside one block and a recovery does not, that the block coming out is
 * the block before the one going in, that the gain multiplying the samples is
 * a straight line with no step in it anywhere including across a block
 * boundary, and that the line stays inside the clamps while the state behind
 * it is free to run past them.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "agc.h"

/** math.h does not have to define M_PI, and under -std=c99 it often does not. */
#define TEST_PI (3.14159265358979323846)

/* The project's loop: 8 kHz, hold -20 dBFS, clamp -20 .. +40 dB, 50 ms up. */
#define SPEC_RATE (8000.0)
#define SPEC_FULL_SCALE (32768.0f)
#define SPEC_TARGET_DBFS (-20.0f)
#define SPEC_MIN_GAIN_DB (-20.0f)
#define SPEC_MAX_GAIN_DB (40.0f)
#define SPEC_TAU_S (0.050f)

#define BLOCK (AGC_BLOCK_SAMPLES)
#define BLOCK_SECONDS ((double)BLOCK / SPEC_RATE)

static int failures = 0;

#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    if (!(cond)) {                                                             \
      printf("  FAIL  %s  (%s:%d)\n", (msg), __FILE__, __LINE__);              \
      failures++;                                                              \
    } else {                                                                   \
      printf("  ok    %s\n", (msg));                                           \
    }                                                                          \
  } while (0)

/** Decibels to a plain ratio, in double, written out rather than shared. */
static double ratio_of_db(double db) { return pow(10.0, db / 20.0); }

/** The amplitude a level in dBFS names, on the wire's scale. */
static double amplitude_of_dbfs(double dbfs) {
  return (double)SPEC_FULL_SCALE * ratio_of_db(dbfs);
}

/** The level an amplitude sits at, in dBFS. */
static double dbfs_of_amplitude(double amplitude) {
  return 20.0 * log10(amplitude / (double)SPEC_FULL_SCALE);
}

static agc_spec_t project_spec(void) {
  agc_spec_t spec;
  spec.full_scale = SPEC_FULL_SCALE;
  spec.target_rms_dbfs = SPEC_TARGET_DBFS;
  spec.min_gain_db = SPEC_MIN_GAIN_DB;
  spec.max_gain_db = SPEC_MAX_GAIN_DB;
  spec.release_alpha = agc_release_alpha(SPEC_TAU_S, (float)SPEC_RATE);
  spec.warmup_blocks = 0u;
  return spec;
}

/** One instance, reinitialised per test - it is only 320 bytes, but a static
 *  keeps the tests off a stack that some of them fill with block histories. */
static agc_t agc;

/** Every block in these tests is one amplitude held flat, because then the
 *  RMS is that amplitude exactly and every expected number below can be
 *  written down rather than measured. The loop has no opinion about spectrum;
 *  test_level_steps() uses a real tone to confirm that. */
static void fill_block(float* block, double amplitude) {
  for (size_t idx = 0u; idx < (size_t)BLOCK; idx++) {
    block[idx] = (float)amplitude;
  }
}

/** RMS of a block, in double, for checking what came out. */
static double block_rms(const float* block) {
  double sum_sq = 0.0;
  for (size_t idx = 0u; idx < (size_t)BLOCK; idx++) {
    sum_sq += (double)block[idx] * (double)block[idx];
  }
  return sqrt(sum_sq / (double)BLOCK);
}

/**
 * @brief Push a flat block of the given amplitude and throw the output away.
 *
 * @param amplitude Amplitude to fill the block with.
 * @return True if a block came out.
 */
static int push_flat(double amplitude) {
  float in[BLOCK];
  float out[BLOCK];
  fill_block(in, amplitude);
  return agc_process(&agc, in, out) ? 1 : 0;
}

/** Push flat blocks until the loop has settled on them. 200 blocks is 1.6 s,
 *  about 32 time constants, so what is left of any transient is far below the
 *  tolerances used here. */
static void settle_at(double amplitude, size_t blocks) {
  for (size_t n = 0u; n < blocks; n++) {
    (void)push_flat(amplitude);
  }
}

/* ------------------------------------------------------ the reference model */

/**
 * The same loop, written the long way round.
 *
 * The library folds a whole block of the per-sample recursion into one
 * coefficient. This does not: it runs the recursion literally, AGC_BLOCK_SAMPLES
 * steps of g += alpha * (target - g), in double. If the two agree then the fold
 * is right, and if the fold were dropped - the coefficient applied once per
 * block instead - they would disagree by a factor of the block size on the
 * first block and never converge back.
 *
 * Deliberately not sharing any code with the library, including the clamp and
 * the ramp.
 */
typedef struct {
  double alpha;
  double target_rms;
  double min_gain;
  double max_gain;
  double gain;
  double applied;
  double pending[BLOCK];
  int have_pending;
} ref_t;

static void ref_init(ref_t* ref, const agc_spec_t* spec) {
  memset(ref, 0, sizeof(*ref));
  ref->alpha = (double)spec->release_alpha;
  ref->target_rms = (double)spec->full_scale * ratio_of_db(spec->target_rms_dbfs);
  ref->min_gain = ratio_of_db(spec->min_gain_db);
  ref->max_gain = ratio_of_db(spec->max_gain_db);
  ref->gain = 1.0;
  ref->applied = 1.0;
  ref->have_pending = 0;
}

static int ref_process(ref_t* ref, const float* in, double* out) {
  double sum_sq = 0.0;
  size_t idx;

  for (idx = 0u; idx < (size_t)BLOCK; idx++) {
    sum_sq += (double)in[idx] * (double)in[idx];
  }

  {
    const double rms = sqrt(sum_sq / (double)BLOCK);
    const double floor_rms = (double)SPEC_FULL_SCALE * 1.0e-9;
    const double measured = (rms > floor_rms) ? rms : floor_rms;
    const double target = ref->target_rms / measured;

    if (target <= ref->gain) {
      ref->gain = target;
    } else {
      /* One step per sample, which is what release_alpha describes. */
      for (idx = 0u; idx < (size_t)BLOCK; idx++) {
        ref->gain += ref->alpha * (target - ref->gain);
      }
    }
  }

  {
    int emitted = 0;

    if (ref->have_pending) {
      double end = ref->gain;
      if (end < ref->min_gain) {
        end = ref->min_gain;
      }
      if (end > ref->max_gain) {
        end = ref->max_gain;
      }

      for (idx = 0u; idx < (size_t)BLOCK; idx++) {
        const double fraction = (double)(idx + 1u) / (double)BLOCK;
        out[idx] = ref->pending[idx] * (ref->applied + fraction * (end - ref->applied));
      }

      ref->applied = end;
      emitted = 1;
    }

    for (idx = 0u; idx < (size_t)BLOCK; idx++) {
      ref->pending[idx] = (double)in[idx];
    }
    ref->have_pending = 1;

    return emitted;
  }
}

/* ------------------------------------------------------------------- tests */

static void test_release_alpha(void) {
  const float alpha = agc_release_alpha(SPEC_TAU_S, (float)SPEC_RATE);

  printf("\nagc_release_alpha\n");

  /* 1 - exp(-1 / (0.050 * 8000)) = 1 - exp(-1/400). */
  CHECK(fabs((double)alpha - 0.0024968795) < 1.0e-7,
        "50 ms at 8 kHz is 0.002496 per sample");

  CHECK(agc_release_alpha(0.0f, 8000.0f) == 0.0f, "rejects a zero time constant");
  CHECK(agc_release_alpha(-1.0f, 8000.0f) == 0.0f, "rejects a negative time constant");
  CHECK(agc_release_alpha(0.05f, 0.0f) == 0.0f, "rejects a zero sample rate");

  /* Longer constants move less per sample; shorter ones move more. */
  CHECK(agc_release_alpha(0.150f, 8000.0f) < alpha, "150 ms is slower than 50 ms");
  CHECK(agc_release_alpha(0.010f, 8000.0f) > alpha, "10 ms is faster than 50 ms");
}

static void test_spec_validation(void) {
  agc_spec_t spec;
  float in[BLOCK];
  float out[BLOCK];

  printf("\nspec validation\n");

  spec = project_spec();
  CHECK(agc_init(NULL, &spec) == AGC_ERR_NULL, "NULL instance");
  CHECK(agc_init(&agc, NULL) == AGC_ERR_NULL, "NULL spec");

  spec = project_spec();
  spec.full_scale = 0.0f;
  CHECK(agc_init(&agc, &spec) == AGC_ERR_FULL_SCALE, "zero full scale");

  spec = project_spec();
  spec.target_rms_dbfs = 6.0f;
  CHECK(agc_init(&agc, &spec) == AGC_ERR_TARGET, "a target above full scale");

  spec = project_spec();
  spec.min_gain_db = 40.0f;
  spec.max_gain_db = -20.0f;
  CHECK(agc_init(&agc, &spec) == AGC_ERR_GAIN_RANGE, "gain limits out of order");

  spec = project_spec();
  spec.release_alpha = 1.5f;
  CHECK(agc_init(&agc, &spec) == AGC_ERR_ALPHA, "a release coefficient above one");

  spec = project_spec();
  spec.release_alpha = -0.1f;
  CHECK(agc_init(&agc, &spec) == AGC_ERR_ALPHA, "a negative release coefficient");

  /* The last refusal above left the instance inert. Audio must come back
   * unchanged - one block late, but unchanged - rather than scaled by
   * whatever was in the struct when the check failed. */
  fill_block(in, 1234.0);
  CHECK(!agc_process(&agc, in, out), "a refused instance emits nothing on the first block");
  CHECK(agc_process(&agc, in, out), "and a block on the second");
  CHECK(fabs((double)out[0] - 1234.0) < 1.0e-6 &&
            fabs((double)out[BLOCK - 1] - 1234.0) < 1.0e-6,
        "which is the input, unscaled");

  spec = project_spec();
  CHECK(agc_init(&agc, &spec) == AGC_OK, "and the project spec is accepted");

  CHECK(!agc_process(NULL, in, out), "agc_process rejects a NULL instance");
  CHECK(!agc_process(&agc, NULL, out), "agc_process rejects a NULL input");
  CHECK(!agc_process(&agc, in, NULL), "agc_process rejects a NULL output");
  CHECK(agc_gain(NULL) == 1.0f, "agc_gain of NULL is unity");
  CHECK(agc_input_rms(NULL) == 0.0f, "agc_input_rms of NULL is zero");
}

static void test_matches_per_sample_reference(void) {
  agc_spec_t spec = project_spec();
  ref_t ref;
  float in[BLOCK];
  float out[BLOCK];
  double ref_out[BLOCK];
  double worst_gain = 0.0;
  double worst_sample = 0.0;
  int emission_agrees = 1;
  size_t n;

  printf("\nagainst a per-sample reference\n");

  CHECK(agc_init(&agc, &spec) == AGC_OK, "initialised");
  ref_init(&ref, &spec);

  /* A level that steps around enough to exercise both directions and both
   * clamps: quiet enough to want the ceiling, loud enough to want the floor. */
  for (n = 0u; n < 400u; n++) {
    static const double levels_dbfs[] = {-45.0, -6.0, -70.0, -20.0, -2.0, -55.0};
    const double amplitude = amplitude_of_dbfs(levels_dbfs[(n / 17u) % 6u]);
    int emitted;

    fill_block(in, amplitude);
    emitted = agc_process(&agc, in, out) ? 1 : 0;

    if (emitted != ref_process(&ref, in, ref_out)) {
      emission_agrees = 0;
    }

    if (emitted) {
      size_t idx;
      const double gain_error =
          fabs((double)agc_gain(&agc) - ref.applied) / ref.applied;
      if (gain_error > worst_gain) {
        worst_gain = gain_error;
      }
      for (idx = 0u; idx < (size_t)BLOCK; idx++) {
        const double scale = fabs(ref_out[idx]) + 1.0;
        const double error = fabs((double)out[idx] - ref_out[idx]) / scale;
        if (error > worst_sample) {
          worst_sample = error;
        }
      }
    }
  }

  CHECK(emission_agrees, "the two agree on which blocks come out");

  {
    char msg[96];

    /* The library works in float and this reference in double, and the ramp
     * reaches each sample's gain by accumulating a step rather than by
     * multiplying, so a few parts in ten million is the floor here. What is
     * being tested is that the fold is the same MODEL, and getting it wrong -
     * one step per block instead of AGC_BLOCK_SAMPLES - is an error of tens of
     * percent, not of parts per million. */
    sprintf(msg, "the folded release tracks the per-sample recursion (%.1e)",
            worst_gain);
    CHECK(worst_gain < 1.0e-4, msg);

    sprintf(msg, "and the samples that come out match (%.1e)", worst_sample);
    CHECK(worst_sample < 1.0e-4, msg);
  }
}

static void test_steady_state(void) {
  agc_spec_t spec = project_spec();
  float in[BLOCK];
  float out[BLOCK];
  static const double levels_dbfs[] = {-50.0, -35.0, -20.0, -10.0, -3.0};
  size_t which;

  printf("\nsteady state\n");

  for (which = 0u; which < 5u; which++) {
    const double amplitude = amplitude_of_dbfs(levels_dbfs[which]);
    double out_dbfs;
    char msg[96];

    CHECK(agc_init(&agc, &spec) == AGC_OK, "initialised");
    settle_at(amplitude, 400u);

    fill_block(in, amplitude);
    (void)agc_process(&agc, in, out);
    out_dbfs = dbfs_of_amplitude(block_rms(out));

    sprintf(msg, "%.0f dBFS in settles to -20 dBFS out (%.3f)",
            levels_dbfs[which], out_dbfs);
    CHECK(fabs(out_dbfs - (double)SPEC_TARGET_DBFS) < 0.01, msg);

    /* And the loop says so itself: what it reports as the input level is the
     * level that went in, and its gain closes the gap to the target. */
    CHECK(fabs(dbfs_of_amplitude((double)agc_input_rms(&agc)) - levels_dbfs[which]) < 0.01,
          "  agc_input_rms reports the level it measured");
    CHECK(fabs(20.0 * log10((double)agc_gain(&agc)) -
               ((double)SPEC_TARGET_DBFS - levels_dbfs[which])) < 0.01,
          "  and agc_gain is the difference to the target");
  }
}

static void test_release_time_constant(void) {
  agc_spec_t spec = project_spec();
  const double loud = amplitude_of_dbfs(-20.0);  /* wants unity gain */
  const double quiet = amplitude_of_dbfs(-40.0); /* wants +20 dB, so 10x */
  const double start_gain = 1.0;
  const double end_gain = 10.0;
  double previous = start_gain;
  double crossed_at = -1.0;
  double tau_ms;
  size_t n;
  char msg[96];

  printf("\nrelease time constant\n");

  CHECK(agc_init(&agc, &spec) == AGC_OK, "initialised");
  settle_at(loud, 400u);
  CHECK(fabs((double)agc_gain(&agc) - start_gain) < 1.0e-4, "settled at unity gain");

  /* One time constant is where the gain has covered 1 - 1/e of the way from
   * where it started to where it is heading. Found by interpolating between
   * the two blocks it happens between, so the answer is not quantised to the
   * 8 ms the blocks arrive on. */
  {
    const double threshold = start_gain + (1.0 - exp(-1.0)) * (end_gain - start_gain);

    for (n = 1u; n <= 40u; n++) {
      double gain;
      (void)push_flat(quiet);
      gain = (double)agc_gain(&agc);

      if ((crossed_at < 0.0) && (gain >= threshold)) {
        crossed_at = (double)n - (gain - threshold) / (gain - previous);
      }
      previous = gain;
    }
  }

  CHECK(crossed_at > 0.0, "the gain crossed one time constant");
  tau_ms = crossed_at * BLOCK_SECONDS * 1000.0;

  sprintf(msg, "measured release is 50 ms (%.2f ms, %.3f blocks)", tau_ms, crossed_at);
  CHECK(fabs(tau_ms - 50.0) < 1.0, msg);

  /* The mistake this is really here to catch: applying the per-sample
   * coefficient once per block instead of folding it, which would give
   * 8 ms / -ln(1 - 0.002496) = 3.2 seconds. */
  CHECK(tau_ms < 200.0, "and not the 3.2 s a per-block coefficient would give");

  /* Having got there, it stays there. */
  settle_at(quiet, 400u);
  CHECK(fabs((double)agc_gain(&agc) - end_gain) < 1.0e-3, "and it converges on the target");
}

static void test_attack_is_immediate(void) {
  agc_spec_t spec = project_spec();
  const double quiet = amplitude_of_dbfs(-40.0);
  const double loud = amplitude_of_dbfs(-10.0);
  float in[BLOCK];
  float out[BLOCK];
  double after_one_block;

  printf("\nattack\n");

  CHECK(agc_init(&agc, &spec) == AGC_OK, "initialised");
  settle_at(quiet, 400u);
  CHECK(fabs((double)agc_gain(&agc) - 10.0) < 1.0e-3, "settled at +20 dB on the quiet signal");

  /* One loud block. The gain it asks for is 10 dB below unity, which is a
   * reduction, so the whole change lands at once. */
  fill_block(in, loud);
  (void)agc_process(&agc, in, out);
  after_one_block = (double)agc_gain(&agc);

  CHECK(fabs(20.0 * log10(after_one_block) - (-10.0)) < 1.0e-3,
        "one loud block moves the gain the whole way, in one block");

  /* The block that came out is the last QUIET one, ramped down to meet the
   * loud block that is coming - which is the lookahead. So the ramp ends at
   * the new gain even though the samples in it are still the old level. */
  CHECK(fabs((double)out[BLOCK - 1] / quiet - after_one_block) < 1.0e-4,
        "and the emitted block's ramp already ends there");

  /* The next block out is the loud one, flat at the new gain, on target. */
  fill_block(in, loud);
  (void)agc_process(&agc, in, out);
  CHECK(fabs(dbfs_of_amplitude(block_rms(out)) - (double)SPEC_TARGET_DBFS) < 0.01,
        "so the loud block itself comes out on target, with no overshoot");
}

static void test_pipeline_delay(void) {
  agc_spec_t spec = project_spec();
  float in[BLOCK];
  float out[BLOCK];
  size_t idx;
  int monotone = 1;
  int recovered = 1;

  printf("\none block of lookahead\n");

  CHECK(agc_init(&agc, &spec) == AGC_OK, "initialised");

  /* Nothing at all comes out of the first block: there is no previous block
   * for its gain to be aimed at yet. */
  fill_block(in, amplitude_of_dbfs(-30.0));
  CHECK(!agc_process(&agc, in, out), "the first block emits nothing");

  /* Mark the second block so it is distinguishable from the first. */
  for (idx = 0u; idx < (size_t)BLOCK; idx++) {
    in[idx] = (float)amplitude_of_dbfs(-6.0);
  }
  CHECK(agc_process(&agc, in, out), "the second block emits one");

  /* What came out is the FIRST block - the -30 dBFS one - scaled. Divide the
   * gain back out and the level that appears is the first block's, not the
   * second's. */
  {
    const double first = amplitude_of_dbfs(-30.0);
    double previous_gain = (double)out[0] / first;

    /* The second block is much louder than the first, so this ramp runs
     * downwards - the reduction is applied to the block BEFORE the loud one,
     * which is the whole point of holding a block back. */
    for (idx = 1u; idx < (size_t)BLOCK; idx++) {
      const double gain = (double)out[idx] / first;
      if (gain >= previous_gain) {
        monotone = 0;
      }
      previous_gain = gain;
      if (fabs((double)out[idx] / gain - first) > 1.0e-3) {
        recovered = 0;
      }
    }
  }

  CHECK(recovered, "the emitted samples are the PREVIOUS block's, scaled");
  CHECK(monotone, "by a gain that falls steadily across the block");

  /* And the ramp ended on the gain the second block asked for, not the
   * first's: the destination comes from the block that follows. */
  CHECK(fabs(20.0 * log10((double)out[BLOCK - 1] / amplitude_of_dbfs(-30.0)) -
             ((double)SPEC_TARGET_DBFS - (-6.0))) < 1.0e-3,
        "aimed at the gain the SECOND block asked for");

  /* agc_input_rms describes the block that came out, not the one going in. */
  CHECK(fabs(dbfs_of_amplitude((double)agc_input_rms(&agc)) - (-30.0)) < 0.01,
        "and agc_input_rms describes the block that came out");
}

static void test_ramp_is_a_line(void) {
  agc_spec_t spec = project_spec();
  static const double levels_dbfs[] = {-60.0, -3.0, -80.0, -25.0, -1.0, -70.0};
  float in[BLOCK];
  float out[BLOCK];
  float previous_in[BLOCK];
  double last_gain = 1.0;
  double last_scale = 2.0;
  double worst_step_error = 0.0;
  double worst_boundary_error = 0.0;
  double lowest = 1.0e30;
  double highest = 0.0;
  size_t n;

  printf("\nthe applied gain is a line, inside the clamps\n");

  CHECK(agc_init(&agc, &spec) == AGC_OK, "initialised");
  fill_block(previous_in, amplitude_of_dbfs(-20.0));

  for (n = 0u; n < 300u; n++) {
    /* Change level every few blocks, hard, in both directions. */
    const double amplitude = amplitude_of_dbfs(levels_dbfs[(n / 5u) % 6u]);
    size_t idx;

    fill_block(in, amplitude);

    if (agc_process(&agc, in, out)) {
      double gains[BLOCK];
      double step;
      double scale;

      for (idx = 0u; idx < (size_t)BLOCK; idx++) {
        gains[idx] = (double)out[idx] / (double)previous_in[idx];
        if (gains[idx] < lowest) {
          lowest = gains[idx];
        }
        if (gains[idx] > highest) {
          highest = gains[idx];
        }
      }

      /* Equally spaced, which is what makes it a straight line rather than a
       * curve or a line clipped flat where it met a rail. Measured against the
       * size of the gain rather than against the size of the step, because a
       * settled loop has a step of exactly zero and everything is a large
       * relative error next to that. */
      scale = fabs(gains[0]) + fabs(gains[BLOCK - 1u]) + 1.0e-12;
      step = gains[1] - gains[0];
      for (idx = 1u; idx < (size_t)BLOCK; idx++) {
        const double error = fabs((gains[idx] - gains[idx - 1u]) - step) / scale;
        if (error > worst_step_error) {
          worst_step_error = error;
        }
      }

      /* And the join to the previous block is one step of the same size, so
       * there is no discontinuity hiding on the boundary.
       *
       * Measured against the larger of the two blocks' gain scales. The error
       * being looked for is a rounding of the PREVIOUS block's ramp, and that
       * ramp may have run from a far larger gain than the block now being
       * measured sits at - dividing by this block's scale alone would report a
       * rounding of 10 as a large error against a gain of 0.1. */
      if (n > 0u) {
        const double denominator = (scale > last_scale) ? scale : last_scale;
        const double error = fabs((gains[0] - last_gain) - step) / denominator;
        if (error > worst_boundary_error) {
          worst_boundary_error = error;
        }
      }

      last_gain = gains[BLOCK - 1u];
      last_scale = scale;
    }

    memcpy(previous_in, in, sizeof(in));
  }

  {
    /* The rails are held to float precision, not to the last bit: the ramp
     * reaches each sample's gain by accumulating a step, so the value at the
     * far end can miss the endpoint it was aimed at by an ulp or two. On a
     * gain of 100 that is seven parts in ten million, or 0.00006 dB. */
    const double slack = 1.0e-5;
    char msg[96];

    sprintf(msg, "every step within a block is the same size (%.1e)", worst_step_error);
    CHECK(worst_step_error < 1.0e-6, msg);

    sprintf(msg, "and the step across a block boundary is too (%.1e)", worst_boundary_error);
    CHECK(worst_boundary_error < 1.0e-6, msg);

    sprintf(msg, "no sample was scaled below -20 dB (lowest %.4f dB)",
            20.0 * log10(lowest));
    CHECK(lowest >= ratio_of_db((double)SPEC_MIN_GAIN_DB) * (1.0 - slack), msg);

    sprintf(msg, "and none above +40 dB (highest %.4f dB)", 20.0 * log10(highest));
    CHECK(highest <= ratio_of_db((double)SPEC_MAX_GAIN_DB) * (1.0 + slack), msg);
  }
}

static void test_clamps(void) {
  agc_spec_t spec = project_spec();
  const double max_gain = ratio_of_db((double)SPEC_MAX_GAIN_DB);
  const double min_gain = ratio_of_db((double)SPEC_MIN_GAIN_DB);

  printf("\nclamps\n");

  /* Silence. The measured level collapses, the ratio it asks for is enormous,
   * and the applied gain has to sit on the ceiling and stay there. */
  CHECK(agc_init(&agc, &spec) == AGC_OK, "initialised");
  settle_at(0.0, 200u);
  CHECK(fabs((double)agc_gain(&agc) - max_gain) < 1.0e-6,
        "silence pins the applied gain at exactly +40 dB");
  CHECK((double)agc.gain > max_gain * 100.0,
        "while the state behind it has run far past the ceiling");

  /* And it comes straight back, because the attack does not have to unwind
   * what the state wound up. One block, not a slow walk down. */
  (void)push_flat(amplitude_of_dbfs(-20.0));
  CHECK(fabs((double)agc.gain - 1.0) < 1.0e-4,
        "and one block of real audio brings the state straight back to unity");

  /* Loud. +20 dBFS in wants -40 dB of gain; the floor is -20 dB. */
  CHECK(agc_init(&agc, &spec) == AGC_OK, "reinitialised");
  settle_at(amplitude_of_dbfs(20.0), 200u);
  CHECK(fabs((double)agc_gain(&agc) - min_gain) < 1.0e-6,
        "an overloaded input floors the applied gain at exactly -20 dB");
  CHECK((double)agc.gain < min_gain,
        "while the state sits below it");

  /* The floor is a floor, not a target: the output stays above -20 dBFS
   * because the loop is not allowed to pull it down any further. */
  {
    float in[BLOCK];
    float out[BLOCK];
    fill_block(in, amplitude_of_dbfs(20.0));
    (void)agc_process(&agc, in, out);
    CHECK(dbfs_of_amplitude(block_rms(out)) > (double)SPEC_TARGET_DBFS,
          "so the output stays above target rather than reaching it");
  }
}

static void test_warmup_and_reset(void) {
  agc_spec_t spec = project_spec();
  const double amplitude = amplitude_of_dbfs(-60.0);
  float in[BLOCK];
  float out[BLOCK];
  int unscaled = 1;
  size_t n;

  printf("\nwarmup and reset\n");

  spec.warmup_blocks = 4u;
  CHECK(agc_init(&agc, &spec) == AGC_OK, "initialised with a four block warmup");

  /* Blocks 0..3 are the warmup. Block 0 emits nothing; blocks 1, 2 and 3 emit
   * blocks 0, 1 and 2 at unity, because the gain is held rather than adapted.
   * A -60 dBFS input would otherwise have driven it to the ceiling by now. */
  for (n = 0u; n < 4u; n++) {
    fill_block(in, amplitude);
    if (agc_process(&agc, in, out)) {
      size_t idx;
      for (idx = 0u; idx < (size_t)BLOCK; idx++) {
        if (fabs((double)out[idx] - amplitude) > 1.0e-3) {
          unscaled = 0;
        }
      }
    }
  }

  CHECK(unscaled, "the warmup blocks come out untouched");
  CHECK((double)agc_gain(&agc) == 1.0, "with the gain still at unity");

  /* The fifth block is the first one measured, so the loop starts here. */
  fill_block(in, amplitude);
  (void)agc_process(&agc, in, out);
  CHECK((double)agc_gain(&agc) > 1.0, "and the block after the warmup starts the loop");

  /* Reset puts it all back, warmup included. */
  settle_at(amplitude, 200u);
  CHECK((double)agc_gain(&agc) > 1.0, "settled somewhere above unity");

  agc_reset(&agc);
  CHECK((double)agc_gain(&agc) == 1.0, "reset returns the gain to unity");
  CHECK(!push_flat(amplitude), "and drops the block in flight");

  unscaled = 1;
  for (n = 0u; n < 3u; n++) {
    size_t idx;
    fill_block(in, amplitude);
    (void)agc_process(&agc, in, out);
    for (idx = 0u; idx < (size_t)BLOCK; idx++) {
      if (fabs((double)out[idx] - amplitude) > 1.0e-3) {
        unscaled = 0;
      }
    }
  }
  CHECK(unscaled, "and restarts the warmup");
}

static void test_level_steps(void) {
  /* The bench test from the brief: a tone whose volume jumps every half
   * second. Half a second is 62.5 blocks, so 62 is used and the levels change
   * on a block boundary. A real sine rather than a flat block, to confirm the
   * loop does not care what is inside a block, only how loud it is. */
  agc_spec_t spec = project_spec();
  static const double segment_dbfs[] = {-30.0, -6.0, -45.0, -12.0, -30.0, -6.0};
  const size_t blocks_per_segment = 62u;
  float in[BLOCK];
  float out[BLOCK];
  double phase = 0.0;
  size_t segment;
  double worst_settled_error = 0.0;
  double worst_settled_spread = 0.0;
  double worst_excursion_db = -1000.0;
  size_t worst_excursion_blocks = 0u;

  printf("\nlevel steps, half a second apart\n");

  CHECK(agc_init(&agc, &spec) == AGC_OK, "initialised");

  for (segment = 0u; segment < 6u; segment++) {
    /* A 1 kHz tone. Its RMS is the amplitude over root two. */
    const double amplitude = amplitude_of_dbfs(segment_dbfs[segment]) * sqrt(2.0);
    double settled_min = 1.0e30;
    double settled_max = -1.0e30;
    size_t over_target = 0u;
    size_t n;

    for (n = 0u; n < blocks_per_segment; n++) {
      size_t idx;

      for (idx = 0u; idx < (size_t)BLOCK; idx++) {
        in[idx] = (float)(amplitude * sin(phase));
        phase += 2.0 * TEST_PI * 1000.0 / SPEC_RATE;
      }

      if (agc_process(&agc, in, out)) {
        const double level = dbfs_of_amplitude(block_rms(out));

        /* Blocks that came out louder than the target. Counted per segment,
         * because what matters is whether an excursion is one block wide or
         * whether the loop is ringing. */
        if (level > (double)SPEC_TARGET_DBFS + 1.0) {
          over_target++;
          if (level > worst_excursion_db) {
            worst_excursion_db = level;
          }
        }

        /* Settled: the last 20 blocks of the segment, by which point even the
         * slowest release here has had three time constants. */
        if (n >= (blocks_per_segment - 20u)) {
          if (level < settled_min) {
            settled_min = level;
          }
          if (level > settled_max) {
            settled_max = level;
          }
        }
      }
    }

    {
      const double error = fabs(0.5 * (settled_min + settled_max) -
                                (double)SPEC_TARGET_DBFS);
      const double spread = settled_max - settled_min;
      if (error > worst_settled_error) {
        worst_settled_error = error;
      }
      if (spread > worst_settled_spread) {
        worst_settled_spread = spread;
      }
      if (over_target > worst_excursion_blocks) {
        worst_excursion_blocks = over_target;
      }
    }
  }

  {
    char msg[112];

    sprintf(msg, "every segment settles on -20 dBFS (worst error %.3f dB)",
            worst_settled_error);
    CHECK(worst_settled_error < 0.2, msg);

    sprintf(msg, "and holds there rather than oscillating (worst spread %.3f dB)",
            worst_settled_spread);
    CHECK(worst_settled_spread < 0.2, msg);

    /* Every one of these steps is instantaneous and enormous - the -6 dBFS to
     * -45 dBFS one is 39 dB between adjacent samples - so the loop is being
     * asked for something no microphone will ever produce. Even so, at most a
     * single block per step comes out above target.
     *
     * That block is the price of the lookahead, and it is the DOWNWARD steps
     * that cause it, not the upward ones. The ramp on the block being emitted
     * is aimed at the gain the NEXT block asked for; when the next block is
     * much quieter, that gain is much higher, and it gets applied to the tail
     * of the loud block that is still going out. It is bounded by one release
     * step - the gain can only rise 14.8% of the way to the new target in a
     * block - and it lasts exactly one block, 8 ms, because the block after it
     * really is the quiet one. Upward steps have the opposite and desirable
     * behaviour: the reduction has finished ramping before the loud audio
     * arrives, which is what test_attack_is_immediate() measures.
     *
     * On the wire it saturates rather than wraps; see saturate_i16() in
     * main.c. */
    sprintf(msg, "at most one block per step goes above target (%u, worst %.2f dBFS)",
            (unsigned)worst_excursion_blocks, worst_excursion_db);
    CHECK(worst_excursion_blocks <= 1u, msg);
  }
}

int main(void) {
  printf("AGC library tests\n");

  test_release_alpha();
  test_spec_validation();
  test_matches_per_sample_reference();
  test_steady_state();
  test_release_time_constant();
  test_attack_is_immediate();
  test_pipeline_delay();
  test_ramp_is_a_line();
  test_clamps();
  test_warmup_and_reset();
  test_level_steps();

  printf("\n%s: %d failure%s\n", (0 == failures) ? "PASS" : "FAIL", failures,
         (1 == failures) ? "" : "s");
  return (0 == failures) ? 0 : 1;
}
