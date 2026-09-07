/**
 * @file agc.h
 * @brief Block automatic gain control. RMS detector, look-ahead ramp,
 *        immediate attack and asymptotic release.
 *
 *  DEPENDENCIES: the C library's math functions, and nothing else. Not CMSIS,
 *  not the STM32 HAL, not a device header. Link with -lm.
 *
 *  NO ALLOCATION. The block held in flight lives inside agc_t as a plain
 *  array, so an instance is a static, a global, or a local, and its size is
 *  known at compile time.
 *
 *  ## What it does
 *
 *  One block in, the previous block out, scaled to hold a target RMS level.
 *  Per block:
 *
 *    1. Measure the block's RMS.
 *    2. Divide the target RMS by it. That ratio is the gain this block wanted.
 *    3. Move the gain state towards that ratio - instantly downwards, slowly
 *       upwards. See "Attack and release" below.
 *    4. Emit the PREVIOUS block, scaled by a straight line running from the
 *       gain the previous block ended on to the gain this block just asked
 *       for.
 *
 *  Step 4 is why a block is held: the ramp's destination is set by the block
 *  that comes AFTER the one being scaled, so a gain reduction is complete by
 *  the time the loud audio that caused it arrives. That costs one block of
 *  latency - AGC_BLOCK_SAMPLES samples, 8 ms at 8 kHz and 64 samples - and it
 *  is the difference between an AGC that ducks an onset and one that clips it
 *  and then ducks. Anything downstream that compares this output against the
 *  unprocessed input has to delay the input by that block as well, or it is
 *  comparing two different moments.
 *
 *  It also means the gain never steps. Every sample in a block gets its own
 *  gain, one linear-interpolation step apart, and the line is continuous
 *  across the block boundary because each block starts from the value the last
 *  one ended on.
 *
 *  ## Attack and release
 *
 *  Deliberately asymmetric, because getting loud is a problem and getting
 *  quiet is not:
 *
 *    - Gain going DOWN: applied in full, immediately. The state jumps to the
 *      new ratio and the emitted block ramps all the way there. Nothing is
 *      smoothed, because a smoothed attack is an attack that arrives late.
 *
 *    - Gain going UP: the state approaches the ratio asymptotically,
 *      g += alpha * (target - g), so the recovery from a loud passage is a
 *      slow fade up rather than a lunge at the noise floor between words.
 *
 *  release_alpha is given PER SAMPLE, which is where the familiar
 *  alpha = 1 - exp(-1 / (tau * fs)) comes from: 0.002496 is 50 ms at 8 kHz.
 *  The gain state is a per-block quantity, though, so applying that number
 *  once per block would give a time constant AGC_BLOCK_SAMPLES times too long.
 *  agc_init() therefore folds the per-sample recursion over a whole block into
 *  a single coefficient,
 *
 *      A = 1 - (1 - release_alpha)^AGC_BLOCK_SAMPLES
 *
 *  which is exactly where the per-sample recursion lands after a block, and
 *  the requested time constant is the one you get. See agc_release_alpha().
 *
 *  ## Clamping
 *
 *  The APPLIED gain is clamped to min_gain_db .. max_gain_db. The gain STATE
 *  is not: on silence the measured RMS collapses, the ratio goes enormous, and
 *  the state releases up past the ceiling while the applied gain sits pinned
 *  at it.
 *
 *  That is safe here, and it is the immediate attack that makes it safe. An
 *  integrator that can run away needs anti-windup because it has to unwind
 *  before it can respond; this one does not unwind, it jumps. The first block
 *  of real audio asks for a lower gain than the state holds, which is the
 *  attack case, so the state lands on the new ratio in one block from wherever
 *  it had drifted to. The behaviour is indistinguishable from a clamped state
 *  in every case where the state sits above the ceiling, and the arithmetic is
 *  one branch shorter.
 *
 *  Note what the clamp does NOT promise: full scale. A clamped gain applied to
 *  a loud enough block still produces samples past full scale, and it is the
 *  caller's job to saturate rather than wrap when converting them - the output
 *  here is float precisely so that decision stays with the caller.
 *
 *  ## Cost
 *
 *  RAM, per instance, at the default block size: 320 bytes on a 32-bit target,
 *  of which 256 is the block in flight.
 *
 *  CPU, per sample: one multiply-accumulate to measure, and a convert, a
 *  multiply-accumulate and a multiply to apply. Per block: one divide and one
 *  square root. Nothing in the audio path calls libm - the logarithms belong
 *  to the caller's telemetry and the exponentials are paid once in
 *  agc_init().
 *
 *  VERIFICATION: the native test in test/ drives the module with synthetic
 *  level steps and measures what comes out - the settled level, the release
 *  time constant against the requested one, the attack landing inside a single
 *  block, the clamps, the continuity of the ramp across block boundaries, and
 *  the one-block delay - against a reference model written independently of
 *  the implementation. It has NOT been run on hardware.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Block size, fixed at compile time.
 *
 * The default matches BLOCK_SAMPLES in framing.h, which is what the capture
 * path hands over. Override it with -DAGC_BLOCK_SAMPLES=n; src/agc.c fails to
 * compile if it is zero.
 */
#ifndef AGC_BLOCK_SAMPLES
  #define AGC_BLOCK_SAMPLES (64)
#endif

/**
 * What the loop should hold, and how fast it should move.
 *
 * Levels are in dBFS and gains in dB, both relative to full_scale, because
 * that is how the brief states them and how the telemetry reports them. The
 * linear equivalents are worked out once in agc_init().
 */
typedef struct {
  float full_scale;      /**< Amplitude that counts as 0 dBFS. 32768 for int16. */
  float target_rms_dbfs; /**< Level to hold, dBFS. Negative. */
  float min_gain_db;     /**< Applied-gain floor, dB. */
  float max_gain_db;     /**< Applied-gain ceiling, dB. Must exceed the floor. */
  float release_alpha;   /**< Upward smoothing, PER SAMPLE. In 0 .. 1. */
  size_t warmup_blocks;  /**< Blocks to pass at unity before measuring. May be 0. */
} agc_spec_t;

/** Why agc_init() refused. */
typedef enum {
  AGC_OK = 0,
  AGC_ERR_NULL,       /**< Instance or spec pointer was NULL. */
  AGC_ERR_FULL_SCALE, /**< Full scale was not positive and finite. */
  AGC_ERR_TARGET,     /**< Target level was not finite, or was above full scale. */
  AGC_ERR_GAIN_RANGE, /**< Gain limits were not finite or not in order. */
  AGC_ERR_ALPHA       /**< Release coefficient outside 0 .. 1. */
} agc_status_t;

/**
 * A loop instance. Opaque in practice - go through the functions below - but
 * declared here so it can be a static, and so its size is visible.
 */
typedef struct {
  agc_spec_t spec;      /**< The spec this was built for. */
  float target_rms;     /**< target_rms_dbfs as an amplitude. */
  float min_gain;       /**< min_gain_db as a ratio. */
  float max_gain;       /**< max_gain_db as a ratio. */
  float release_block;  /**< Per-sample release folded over a whole block. */
  float gain;           /**< Gain state. Unclamped; see "Clamping" above. */
  float applied;        /**< Clamped gain the last emitted block ended on. */
  float pending[AGC_BLOCK_SAMPLES]; /**< Block waiting for its ramp. */
  float pending_rms;    /**< That block's own RMS, before any gain. */
  float emitted_rms;    /**< RMS of the block last emitted, before any gain. */
  bool have_pending;    /**< False only before the first block has arrived. */
  size_t warmup_left;   /**< Blocks still to pass through untouched. */
} agc_t;

/**
 * @brief Work out the derived constants and clear the state.
 *
 * Call it once, at startup. It is the only function here that touches libm.
 *
 * On any status other than AGC_OK the instance is left inert - unity gain,
 * no measurement - so a caller that ignores the return value gets its audio
 * back unchanged rather than scaled by nonsense. Do not ignore the return
 * value.
 *
 * @param agc Instance to initialise.
 * @param spec What it should do. Copied in; need not outlive the call.
 * @return AGC_OK, or which check failed.
 */
agc_status_t agc_init(agc_t* agc, const agc_spec_t* spec);

/**
 * @brief Forget the audio and the gain, keep the spec.
 *
 * Returns the gain to unity, drops the block in flight and restarts the warmup
 * count. For a discontinuity in the input, where the level either side is not
 * one signal and the loop should not carry a measurement across.
 *
 * @param agc Instance to clear.
 */
void agc_reset(agc_t* agc);

/**
 * @brief Take one block in, give the previous one back scaled.
 *
 * in and out must not overlap: out receives the PREVIOUS call's block, so
 * writing it would destroy the input still being read. Both are
 * AGC_BLOCK_SAMPLES long.
 *
 * The output is not limited to any range - it is float, and a clamped gain on
 * a loud block still exceeds full scale. Saturate on the way to integers.
 *
 * @param agc Initialised instance.
 * @param in This block's samples, on the same scale as spec.full_scale.
 * @param out The previous block, scaled. Untouched when false is returned.
 * @return True if a block was emitted; false only on the first call after an
 *         init or a reset, when there is no previous block to emit yet.
 */
bool agc_process(agc_t* agc, const float* in, float* out);

/**
 * @brief Gain the block that agc_process() just emitted ended on.
 *
 * Clamped, and a ratio rather than dB - 20*log10 of it is what the telemetry
 * record wants. It is the value the ramp finished at, not an average over the
 * block, so it is the loop's current position rather than a description of
 * what the block was scaled by.
 *
 * @param agc Initialised instance.
 * @return The gain, as a ratio. Unity before the first block is emitted.
 */
float agc_gain(const agc_t* agc);

/**
 * @brief RMS of the block that agc_process() just emitted, before the gain.
 *
 * An amplitude on the same scale as spec.full_scale; divide by full_scale and
 * take 20*log10 for dBFS. Measured on the block being emitted, not on the one
 * just pushed in, so it lines up with agc_gain() and with the samples the
 * caller is about to send.
 *
 * @param agc Initialised instance.
 * @return The RMS. Zero before the first block is emitted.
 */
float agc_input_rms(const agc_t* agc);

/**
 * @brief Per-sample release coefficient for a time constant.
 *
 * 1 - exp(-1 / (tau * fs)), the usual one-pole design. For filling in
 * agc_spec_t.release_alpha without hard-coding a number whose provenance is
 * then a comment: agc_release_alpha(0.050f, 8000.0f) is 0.002496.
 *
 * Uses libm, so call it at startup rather than in the audio path.
 *
 * @param tau_seconds Time constant, seconds. Positive.
 * @param sample_rate_hz Sample rate, Hz. Positive.
 * @return The coefficient, in 0 .. 1, or 0 if either argument is unusable.
 */
float agc_release_alpha(float tau_seconds, float sample_rate_hz);

#ifdef __cplusplus
}
#endif
