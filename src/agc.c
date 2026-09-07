/**
 * @file agc.c
 * @brief See agc.h.
 *
 *  Two halves that meet only through the derived constants in agc_t: a setup
 *  half that runs once and is allowed to be slow, and a per-block half that
 *  runs in the audio path and is not. Everything logarithmic - the target
 *  level, the gain limits, the release time constant - is turned into a plain
 *  ratio in agc_init(), so the loop itself is a divide, a square root, and two
 *  arithmetic operations per sample. Nothing below agc_init() calls libm.
 *
 *  The gain is worked out as a ratio rather than as a difference of decibels.
 *  Those are the same number:
 *
 *      gain_db = target_dbfs - rms_dbfs
 *              = 20*log10(target/full_scale) - 20*log10(rms/full_scale)
 *              = 20*log10(target/rms)
 *
 *  so 10^(gain_db/20) is target/rms, and the ratio is what the samples want
 *  multiplying by anyway. Going through decibels would put a log10f and a
 *  powf in every block to arrive back where the divide already was.
 */

#include "agc.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/**
 * The ramp divides by the block length and the mean square divides by it too,
 * so an empty block would divide by zero.
 *
 * A compile-time assertion written the C99 way, as a typedef of an array whose
 * size goes negative when the condition fails. C11's _Static_assert would be
 * clearer, but the native tests build as C99.
 */
typedef char agc_block_must_not_be_empty[((AGC_BLOCK_SAMPLES) >= 1) ? 1 : -1];

/**
 * How far below full scale a block has to measure before its RMS is treated
 * as a floor rather than as a level, so the divide that follows stays finite.
 *
 * -180 dBFS. Nothing physical is down there - it is 90 dB below the least
 * significant bit of the int16 the caller is heading for - so the floor can
 * only ever be hit by exact digital silence or by something within rounding
 * distance of it. The ratio it produces is enormous and the gain state is
 * allowed to hold it; see "Clamping" in agc.h for why that does no harm.
 */
#define AGC_RMS_FLOOR (1.0e-9f)

/** How far along the ramp one sample carries it. See agc_process(). */
#define AGC_RAMP_FRACTION_STEP (1.0f / (float)(AGC_BLOCK_SAMPLES))

/* ------------------------------------------------------------------ setup */

/**
 * @brief Turn a level or a gain in decibels into a plain ratio.
 *
 * @param db Decibels.
 * @return 10^(db/20).
 */
static float db_to_ratio(float db) {
  return powf(10.0f, db / 20.0f);
}

float agc_release_alpha(float tau_seconds, float sample_rate_hz) {
  if (!isfinite(tau_seconds) || !isfinite(sample_rate_hz) ||
      (tau_seconds <= 0.0f) || (sample_rate_hz <= 0.0f)) {
    return 0.0f;
  }
  return 1.0f - expf(-1.0f / (tau_seconds * sample_rate_hz));
}

void agc_reset(agc_t* agc) {
  if (NULL == agc) {
    return;
  }

  agc->gain = 1.0f;
  agc->applied = 1.0f;
  memset(agc->pending, 0, sizeof(agc->pending));
  agc->pending_rms = 0.0f;
  agc->emitted_rms = 0.0f;
  agc->have_pending = false;
  agc->warmup_left = agc->spec.warmup_blocks;
}

/**
 * @brief Leave the instance passing audio through untouched.
 *
 * The state agc_init() falls back to when it refuses a spec: unity gain, no
 * measurement, no warmup. A caller that ignored the return value then gets its
 * audio back one block late and otherwise unchanged, which is a failure that
 * can be heard for what it is rather than one that sounds like a broken loop.
 *
 * @param agc Instance to make inert.
 */
static void agc_make_inert(agc_t* agc) {
  memset(&agc->spec, 0, sizeof(agc->spec));
  agc->target_rms = 0.0f;
  agc->min_gain = 1.0f;
  agc->max_gain = 1.0f;
  agc->release_block = 0.0f;
  agc_reset(agc);
}

agc_status_t agc_init(agc_t* agc, const agc_spec_t* spec) {
  if ((NULL == agc) || (NULL == spec)) {
    if (NULL != agc) {
      agc_make_inert(agc);
    }
    return AGC_ERR_NULL;
  }

  if (!isfinite(spec->full_scale) || (spec->full_scale <= 0.0f)) {
    agc_make_inert(agc);
    return AGC_ERR_FULL_SCALE;
  }

  /* A target above full scale is a target the loop can never settle on: it
   * would ask for gain forever and sit on the ceiling. */
  if (!isfinite(spec->target_rms_dbfs) || (spec->target_rms_dbfs > 0.0f)) {
    agc_make_inert(agc);
    return AGC_ERR_TARGET;
  }

  if (!isfinite(spec->min_gain_db) || !isfinite(spec->max_gain_db) ||
      (spec->min_gain_db >= spec->max_gain_db)) {
    agc_make_inert(agc);
    return AGC_ERR_GAIN_RANGE;
  }

  /* Zero is allowed, and means the gain never rises - a limiter rather than an
   * AGC. One is allowed too, and means the release is as immediate as the
   * attack. Outside that the recursion does not converge. */
  if (!isfinite(spec->release_alpha) || (spec->release_alpha < 0.0f) ||
      (spec->release_alpha > 1.0f)) {
    agc_make_inert(agc);
    return AGC_ERR_ALPHA;
  }

  agc->spec = *spec;
  agc->target_rms = spec->full_scale * db_to_ratio(spec->target_rms_dbfs);
  agc->min_gain = db_to_ratio(spec->min_gain_db);
  agc->max_gain = db_to_ratio(spec->max_gain_db);

  /* The per-sample recursion g += a*(target - g) leaves g a distance
   * (1 - a)^n from the target after n steps, so a whole block of it is one
   * step of size 1 - (1 - a)^AGC_BLOCK_SAMPLES. Folding it here is what makes
   * release_alpha mean what it says: applied once per block instead, the time
   * constant would come out AGC_BLOCK_SAMPLES times too long. */
  agc->release_block = 1.0f - powf(1.0f - spec->release_alpha,
                                   (float)AGC_BLOCK_SAMPLES);

  agc_reset(agc);
  return AGC_OK;
}

/* ------------------------------------------------------------- audio path */

/**
 * @brief Hold a gain inside the configured limits.
 *
 * Applied to what multiplies the samples, never to the state behind it.
 *
 * @param agc Initialised instance.
 * @param gain Gain to clamp, as a ratio.
 * @return The gain, clamped.
 */
static float clamp_gain(const agc_t* agc, float gain) {
  if (gain < agc->min_gain) {
    return agc->min_gain;
  }
  if (gain > agc->max_gain) {
    return agc->max_gain;
  }
  return gain;
}

/**
 * @brief Root mean square of one block.
 *
 * Accumulated in float. The largest sum a full-scale int16 block can reach is
 * 64 * 32768^2, about 6.9e10, which is nowhere near the top of the range, and
 * 64 additions of like-sized terms cost a few parts in ten million - far below
 * anything a level measurement cares about.
 *
 * @param block Samples.
 * @return Their RMS, on the same scale as the samples.
 */
static float block_rms(const float* block) {
  float sum_sq = 0.0f;

  for (size_t idx = 0u; idx < (size_t)AGC_BLOCK_SAMPLES; idx++) {
    sum_sq += block[idx] * block[idx];
  }

  return sqrtf(sum_sq / (float)AGC_BLOCK_SAMPLES);
}

/**
 * @brief Move the gain state towards what this block asked for.
 *
 * Down at once, up slowly. See "Attack and release" in agc.h.
 *
 * @param agc Initialised instance.
 * @param rms This block's measured RMS.
 */
static void agc_advance(agc_t* agc, float rms) {
  const float floor_rms = agc->spec.full_scale * AGC_RMS_FLOOR;
  const float measured = (rms > floor_rms) ? rms : floor_rms;
  const float target = agc->target_rms / measured;

  if (target <= agc->gain) {
    agc->gain = target;
  } else {
    agc->gain += agc->release_block * (target - agc->gain);
  }
}

bool agc_process(agc_t* agc, const float* in, float* out) {
  if ((NULL == agc) || (NULL == in) || (NULL == out)) {
    return false;
  }

  const float rms = block_rms(in);

  /* The block the loop is about to be told about is the one AFTER the block it
   * is about to emit, which is the whole point: a gain reduction is finished
   * ramping by the time the audio that caused it arrives. */
  if (agc->warmup_left > 0u) {
    /* The filter upstream has not filled yet, so this block is not a level -
     * it is a transient. Pass it through and measure nothing. */
    agc->warmup_left--;
    agc->gain = 1.0f;
  } else {
    agc_advance(agc, rms);
  }

  bool emitted = false;

  if (agc->have_pending) {
    /* A straight line from where the last block finished to where this one
     * asks to be, one step per sample, so the gain never steps and the line
     * is continuous across the boundary. Both ends are clamped and the
     * interpolation between them therefore is too - which is not the same as
     * clamping each sample, and is the reason a state that has run far past
     * the ceiling still produces a smooth ramp rather than a jump to it. */
    const float start = agc->applied;
    const float end = clamp_gain(agc, agc->gain);
    const float span = end - start;

    for (size_t idx = 0u; idx < (size_t)AGC_BLOCK_SAMPLES; idx++) {
      /* Each sample's position along the line is computed, not accumulated.
       * Accumulating a step is one instruction cheaper and drifts: the errors
       * of AGC_BLOCK_SAMPLES roundings add up, and on a ramp that starts high
       * and ends low they add up to a fair fraction of where it ends - a few
       * parts in ten thousand, measured. The next block then starts from the
       * exact endpoint, and that difference is a step in the gain at the block
       * boundary, which is the one thing the ramp exists to avoid. Computed
       * this way the worst error anywhere is a single rounding.
       *
       * The reciprocal is a constant, and an exact one whenever the block size
       * is a power of two - which also makes the last sample's fraction
       * exactly 1, so the line ends where it was aimed. */
      const float fraction = (float)(idx + 1u) * AGC_RAMP_FRACTION_STEP;
      out[idx] = agc->pending[idx] * (start + span * fraction);
    }

    agc->applied = end;
    agc->emitted_rms = agc->pending_rms;
    emitted = true;
  }

  memcpy(agc->pending, in, sizeof(agc->pending));
  agc->pending_rms = rms;
  agc->have_pending = true;

  return emitted;
}

float agc_gain(const agc_t* agc) {
  return (NULL != agc) ? agc->applied : 1.0f;
}

float agc_input_rms(const agc_t* agc) {
  return (NULL != agc) ? agc->emitted_rms : 0.0f;
}
