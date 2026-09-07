/**
 * @file fir_bandpass.h
 * @brief Linear-phase FIR bandpass filter. Fixed taps, designed at run time.
 *
 *  DEPENDENCIES: the C library's math functions, and nothing else. Not CMSIS,
 *  not the STM32 HAL, not a device header. Link with -lm.
 *
 *  NO ALLOCATION. The filter owns its coefficients and its sample history as
 *  plain arrays inside fir_bandpass_t, so an instance is a static, a global,
 *  or a local, and its size is known at compile time. Nothing here calls
 *  malloc, and nothing here calls into the operating system.
 *
 *  The tap count is fixed at compile time by FIR_BANDPASS_TAPS. The band is
 *  not: pass fir_bandpass_init() a spec and it designs the coefficients then
 *  and there, which is what lets one build carry several filters over
 *  different bands, or retune one without a reflash.
 *
 *  ## The design
 *
 *  Windowed sinc with a Kaiser window: the ideal bandpass impulse response,
 *  which is the difference of two sincs, multiplied by the Kaiser window whose
 *  beta gives the requested stopband attenuation. Kaiser is the right window
 *  here because it is the one you can dial: beta sets the attenuation and the
 *  tap count sets the transition width, and both follow closed-form estimates,
 *  so the filter that comes out is the filter that was asked for rather than
 *  whatever the window happened to give.
 *
 *  The coefficients are symmetric, h[k] == h[N-1-k], so the phase response is
 *  exactly linear - every frequency is delayed by the same
 *  FIR_BANDPASS_GROUP_DELAY samples and no waveform is smeared. That matters
 *  for an AGC: a level measured on the filtered signal lags the input by a
 *  known, constant amount rather than a frequency-dependent one. It also
 *  halves the multiplies, since only half the coefficients are distinct.
 *
 *  ## Where the band edges sit
 *
 *  The spec names the passband and the skirt width, and the skirt is placed
 *  OUTSIDE the passband. For 300 Hz .. 3000 Hz with a 150 Hz skirt:
 *
 *     0 .......... 150 ...... 300 ................ 3000 ...... 3150 .. 4000
 *     |<- stopband ->|<- skirt ->|<-- passband -->|<- skirt ->|<- stopband ->|
 *                       225                          3075                       <- -6 dB points
 *
 *  so content between 300 Hz and 3000 Hz passes at unity gain, and content at
 *  or beyond 150 Hz and 3150 Hz is attenuated by at least stopband_db. The
 *  -6 dB cutoffs land in the middle of each skirt, at 225 Hz and 3075 Hz.
 *
 *  ## Why the tap count is not the textbook number
 *
 *  Kaiser's order estimate is derived for a LOWPASS, which has one transition
 *  band. A bandpass is the difference of two lowpasses, and in the stopbands
 *  both of them sit in their passbands, so their ripples subtract to nearly
 *  zero but their errors add: worst case 2 x delta, which is 6 dB. The
 *  textbook 195 taps for the spec above therefore measures -58.8 dB, not the
 *  -60 dB asked for.
 *
 *  So the design targets stopband_db + FIR_BANDPASS_DESIGN_MARGIN_DB, both in
 *  the window's beta and in the order estimate, and the requested attenuation
 *  is met with margin instead of missed by a decibel. That is where 217 taps
 *  comes from. The margin was checked against fifteen assorted specs - see
 *  test/test_fir_bandpass.c, which measures the response of the coefficients
 *  this library actually produces rather than trusting the estimate.
 *
 *  ## Cost
 *
 *  RAM, per instance, at the default 217 taps: 2196 bytes on a 32-bit target,
 *  of which 1736 is the sample history and 868 of that is the second copy the
 *  contiguous window costs.
 *
 *  CPU, per sample: 109 multiply-accumulates and 108 adds. At 8 kHz that is
 *  under a million multiply-accumulates a second, which on a 170 MHz
 *  Cortex-M4F should land near 3% of the CPU - ESTIMATED, not measured. Time
 *  it with the DWT counter in instrument.h before quoting it.
 *
 *  Flash: about 2.8 KB for this file, plus about 13 KB of newlib's
 *  double-precision math - pow, and the argument reduction behind sin and cos
 *  - pulled in by the design half and by nothing else. Measured by linking the
 *  firmware with the filter retained. On a 512 KB part that is 3% of flash for
 *  a design that is provably accurate, which is the right way round; if it
 *  ever is not, the single-precision calls are accurate enough to hold 60 dB,
 *  just with much less headroom above the stopband floor.
 *
 *  Time, at startup: fir_bandpass_init() evaluates a few hundred
 *  double-precision sines, which the M4's single-precision FPU cannot help
 *  with, so budget a few milliseconds. Paid once, and it is what keeps the
 *  design error far enough below the stopband floor not to matter.
 *
 *  VERIFICATION: the native test in test/ measures the response of the
 *  designed coefficients across the whole spectrum and checks the passband,
 *  the skirts and the stopbands against the spec, then checks the streaming
 *  path against a direct convolution. It has NOT been run on hardware.
 */

#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Tap count, fixed at compile time.
 *
 * The default is sized for the project's band: 300 Hz .. 3000 Hz at 8 kHz
 * with a 150 Hz skirt and 60 dB of attenuation. Override it with
 * -DFIR_BANDPASS_TAPS=n for a different one; fir_bandpass_design_taps() says
 * what a given spec needs, and fir_bandpass_init() refuses a spec that does
 * not fit rather than quietly delivering less attenuation than asked for.
 *
 * MUST BE ODD. An even-length symmetric FIR is forced to zero at Nyquist and
 * has a half-sample delay; odd lengths have neither problem. src/fir_bandpass.c
 * fails to compile if this is even.
 */
#ifndef FIR_BANDPASS_TAPS
  #define FIR_BANDPASS_TAPS (217)
#endif

/** Distinct coefficients. The other half are the mirror image. */
#define FIR_BANDPASS_HALF_TAPS ((FIR_BANDPASS_TAPS + 1) / 2)

/**
 * Delay through the filter, in samples, the same at every frequency.
 *
 * 108 samples at the default tap count, which is 13.5 ms at 8 kHz. Anything
 * comparing the input to the output - an AGC measuring both levels, say -
 * has to line them up by this much or it is comparing different moments.
 */
#define FIR_BANDPASS_GROUP_DELAY ((FIR_BANDPASS_TAPS - 1) / 2)

/**
 * Added to the requested attenuation before the window and the order are
 * computed. See "Why the tap count is not the textbook number" above; 6 dB is
 * the worst case for two error terms adding.
 */
#define FIR_BANDPASS_DESIGN_MARGIN_DB (6.0)

/**
 * What the filter should do. Held in the filter object, so an instance
 * carries the band it was built for.
 *
 * The skirts sit outside the passband, so the stopbands begin at
 * pass_low_hz - transition_hz and pass_high_hz + transition_hz. Both must
 * land strictly inside 0 .. sample_rate_hz / 2.
 */
typedef struct {
  float sample_rate_hz;  /**< Sample rate, Hz. */
  float pass_low_hz;     /**< Lower passband edge, Hz. Unity gain from here up. */
  float pass_high_hz;    /**< Upper passband edge, Hz. Unity gain to here. */
  float transition_hz;   /**< Skirt width, Hz. How far outside the passband full attenuation takes. */
  float stopband_db;     /**< Attenuation in the stopbands, dB, positive. 20 to 180. */
} fir_bandpass_spec_t;

/** Why fir_bandpass_init() refused. */
typedef enum {
  FIR_BANDPASS_OK = 0,
  FIR_BANDPASS_ERR_NULL,        /**< Filter or spec pointer was NULL. */
  FIR_BANDPASS_ERR_RATE,        /**< Sample rate was not positive and finite. */
  FIR_BANDPASS_ERR_BAND,        /**< Edges out of order, or a skirt running past DC or Nyquist. */
  FIR_BANDPASS_ERR_TRANSITION,  /**< Skirt width was not positive and finite. */
  FIR_BANDPASS_ERR_ATTENUATION, /**< Attenuation outside 20 .. 180 dB. */
  FIR_BANDPASS_ERR_TAPS,        /**< Spec needs more taps than FIR_BANDPASS_TAPS. */
  FIR_BANDPASS_ERR_GAIN         /**< Design produced no usable passband gain. Not expected. */
} fir_bandpass_status_t;

/**
 * A filter instance. Opaque in practice - go through the functions below -
 * but declared here so it can be a static, and so its size is visible.
 *
 * The history is kept twice over, the second copy mirroring the first, which
 * is what lets the newest FIR_BANDPASS_TAPS samples always be read as one
 * contiguous run whatever the write position. That costs 868 bytes and buys a
 * convolution loop with no index wrapping in it.
 */
typedef struct {
  fir_bandpass_spec_t spec;                    /**< The spec this was built for. */
  float coeff[FIR_BANDPASS_HALF_TAPS];         /**< coeff[k] is h[k], and h[N-1-k]. */
  float history[2 * FIR_BANDPASS_TAPS];        /**< Past samples, stored twice. */
  size_t write;                                /**< Next write position, 0 .. taps-1. */
} fir_bandpass_t;

/**
 * @brief Design the coefficients and clear the history.
 *
 * Does all the arithmetic up front. Call it once, at startup, before the
 * audio path is running - it is far too slow for an interrupt, and it leaves
 * the filter unusable if it fails.
 *
 * On any status other than FIR_BANDPASS_OK the filter is zeroed, so a caller
 * that ignores the return value gets silence rather than noise. Do not ignore
 * the return value.
 *
 * @param filter Filter to initialise.
 * @param spec What it should do. Copied into the filter; need not outlive the call.
 * @return FIR_BANDPASS_OK, or which check failed.
 */
fir_bandpass_status_t fir_bandpass_init(fir_bandpass_t* filter,
                                        const fir_bandpass_spec_t* spec);

/**
 * @brief Forget the past samples, keep the coefficients.
 *
 * For a discontinuity in the input - a gain change, a resynchronisation -
 * where the samples either side are not one signal and should not be
 * convolved together.
 *
 * @param filter Filter to clear.
 */
void fir_bandpass_reset(fir_bandpass_t* filter);

/**
 * @brief Filter one sample.
 *
 * The output is FIR_BANDPASS_GROUP_DELAY samples behind the input. The first
 * FIR_BANDPASS_TAPS outputs after an init or a reset are the filter filling
 * up, and are not meaningful.
 *
 * @param filter Initialised filter.
 * @param sample Input sample. Any scaling; the filter is unity gain in band.
 * @return Filtered sample.
 */
float fir_bandpass_tick(fir_bandpass_t* filter, float sample);

/**
 * @brief Filter a block, sample for sample.
 *
 * Identical to calling fir_bandpass_tick() in a loop - the history carries
 * across calls, so consecutive blocks of a stream filter the same as the
 * unbroken stream. in and out may be the same buffer.
 *
 * @param filter Initialised filter.
 * @param in Input samples.
 * @param out Output samples. May alias in.
 * @param count How many.
 */
void fir_bandpass_process(fir_bandpass_t* filter, const float* in, float* out,
                          size_t count);

/**
 * @brief Taps a spec needs, including the design margin.
 *
 * For choosing FIR_BANDPASS_TAPS, or for checking a spec before committing to
 * it. Always odd. Saturates at 1000001 rather than overflowing on an absurdly
 * narrow skirt.
 *
 * @param spec Spec to size. Not validated beyond what the estimate needs.
 * @return Tap count, or 0 if spec is NULL or its numbers are unusable.
 */
size_t fir_bandpass_design_taps(const fir_bandpass_spec_t* spec);

/**
 * @brief Gain at one frequency, in dB, relative to the passband.
 *
 * For checking a design - in a test, or in a startup self-check - not for the
 * audio path. Costs about as much as filtering a sample and uses double
 * precision to stay accurate down in the stopband.
 *
 * @param filter Initialised filter.
 * @param hz Frequency, Hz. Meaningful from 0 to sample_rate_hz / 2.
 * @return Gain in dB, 0 dB being the passband. Floors at -240 dB.
 */
float fir_bandpass_response_db(const fir_bandpass_t* filter, float hz);

#ifdef __cplusplus
}
#endif
