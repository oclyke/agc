/**
 * @file fir_bandpass.c
 * @brief See fir_bandpass.h.
 *
 *  Two halves that meet only through the coefficient array: a design half that
 *  runs once at startup in double precision, and a convolution half that runs
 *  per sample in float. The split is deliberate. Design error has to sit far
 *  below the stopband floor or it becomes the stopband floor, and doubles are
 *  cheap when they run once; the audio path has to be quick, and the M4's FPU
 *  is single precision, so doubles there would be soft-float and ruinous.
 */

#include "fir_bandpass.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/**
 * The fold in fir_bandpass_tick() pairs tap k with tap N-1-k and leaves one
 * unpaired tap in the middle, which only exists when the count is odd. An even
 * count would also put a zero at Nyquist and half a sample of delay in the
 * phase response. So: odd, and at least one pair plus a centre.
 *
 * These are compile-time assertions written the C99 way, as typedefs of an
 * array whose size goes negative when the condition fails. C11's
 * _Static_assert would be clearer, but the native tests build as C99.
 */
typedef char fir_bandpass_taps_must_be_odd[((FIR_BANDPASS_TAPS) % 2 == 1) ? 1 : -1];
typedef char fir_bandpass_taps_must_be_three_or_more[((FIR_BANDPASS_TAPS) >= 3) ? 1 : -1];

/** math.h does not have to define M_PI, and under -std=c99 it often does not. */
#define FIR_PI (3.14159265358979323846)

/** Where the order estimate gives up, rather than overflowing on a hair-thin skirt. */
#define FIR_TAPS_SATURATION (1.0e6)

/* ----------------------------------------------------------------- design */

/**
 * @brief Modified Bessel function of the first kind, order zero.
 *
 * By its series, I0(x) = sum over k of ((x/2)^k / k!)^2, each term reached
 * from the last by multiplying by (x/2)^2 / k^2 - so no factorials are ever
 * formed and nothing overflows on the way. The terms grow until k is about
 * x/2 and then fall away quickly; at the largest beta this library will ask
 * for, about 19.5, they are done inside 40 iterations.
 *
 * @param x Argument, non-negative.
 * @return I0(x).
 */
static double bessel_i0(double x) {
  const double quarter_sq = 0.25 * x * x;
  double term = 1.0;
  double sum = 1.0;

  for (unsigned int k = 1u; k < 64u; k++) {
    term *= quarter_sq / ((double)k * (double)k);
    sum += term;
    if (term < 1.0e-18 * sum) {
      break;
    }
  }
  return sum;
}

/**
 * @brief Normalised sinc, sin(pi x) / (pi x).
 *
 * The exact comparison against zero is correct rather than sloppy: this is
 * only ever called with x an integer multiple of a constant, and the only way
 * to land on the removable singularity is to land on it exactly. A tolerance
 * would return 1.0 for arguments that are merely small, which is wrong.
 *
 * @param x Argument.
 * @return sinc(x), and 1.0 at x == 0.
 */
static double sinc_pi(double x) {
  if (0.0 == x) {
    return 1.0;
  }
  const double scaled = FIR_PI * x;
  return sin(scaled) / scaled;
}

/**
 * @brief Kaiser window parameter for a stopband attenuation.
 *
 * Kaiser's empirical fit. Below 21 dB the rectangular window already does
 * better than asked, so beta is zero there.
 *
 * @param attenuation_db Stopband attenuation, dB, positive.
 * @return beta.
 */
static double kaiser_beta(double attenuation_db) {
  if (attenuation_db > 50.0) {
    return 0.1102 * (attenuation_db - 8.7);
  }
  if (attenuation_db >= 21.0) {
    return 0.5842 * pow(attenuation_db - 21.0, 0.4) + 0.07886 * (attenuation_db - 21.0);
  }
  return 0.0;
}

/**
 * @brief Kaiser's tap count estimate, forced odd.
 *
 * order = (A - 8) / (2.285 * delta omega), with the transition width in
 * radians per sample. The caller is expected to have already added the
 * bandpass design margin to the attenuation.
 *
 * @param attenuation_db Attenuation to design for, dB.
 * @param transition_hz Transition width, Hz.
 * @param sample_rate_hz Sample rate, Hz.
 * @return Odd tap count, saturating at 1000001.
 */
static size_t kaiser_taps(double attenuation_db, double transition_hz,
                          double sample_rate_hz) {
  const double transition_rad = 2.0 * FIR_PI * transition_hz / sample_rate_hz;
  double order = ceil((attenuation_db - 8.0) / (2.285 * transition_rad));

  if (!(order > 1.0)) {
    order = 1.0; /* also catches a NaN */
  }
  if (order > FIR_TAPS_SATURATION) {
    order = FIR_TAPS_SATURATION;
  }

  size_t taps = (size_t)order + 1u;
  if (0u == (taps % 2u)) {
    taps += 1u;
  }
  return taps;
}

size_t fir_bandpass_design_taps(const fir_bandpass_spec_t* spec) {
  if (NULL == spec) {
    return 0u;
  }

  const double sample_rate = (double)spec->sample_rate_hz;
  const double transition = (double)spec->transition_hz;
  const double attenuation = (double)spec->stopband_db;

  if (!(sample_rate > 0.0) || !isfinite(sample_rate)) {
    return 0u;
  }
  if (!(transition > 0.0) || !isfinite(transition)) {
    return 0u;
  }
  if (!(attenuation >= 20.0) || !(attenuation <= 180.0)) {
    return 0u;
  }
  return kaiser_taps(attenuation + FIR_BANDPASS_DESIGN_MARGIN_DB, transition, sample_rate);
}

fir_bandpass_status_t fir_bandpass_init(fir_bandpass_t* filter,
                                        const fir_bandpass_spec_t* spec) {
  if (NULL == filter || NULL == spec) {
    return FIR_BANDPASS_ERR_NULL;
  }

  /* Zero first, so every failure below leaves a filter that outputs silence
   * rather than one holding half a design. */
  memset(filter, 0, sizeof *filter);

  const double sample_rate = (double)spec->sample_rate_hz;
  const double pass_low = (double)spec->pass_low_hz;
  const double pass_high = (double)spec->pass_high_hz;
  const double transition = (double)spec->transition_hz;
  const double attenuation = (double)spec->stopband_db;

  /* Written as !(x > 0) rather than (x <= 0) throughout, because a NaN fails
   * every comparison and so has to fail an inverted one to be rejected. */
  if (!(sample_rate > 0.0) || !isfinite(sample_rate)) {
    return FIR_BANDPASS_ERR_RATE;
  }
  if (!(transition > 0.0) || !isfinite(transition)) {
    return FIR_BANDPASS_ERR_TRANSITION;
  }
  if (!(attenuation >= 20.0) || !(attenuation <= 180.0)) {
    return FIR_BANDPASS_ERR_ATTENUATION;
  }
  if (!(pass_low < pass_high)) {
    return FIR_BANDPASS_ERR_BAND;
  }
  /* The skirts sit outside the passband, so they, not the passband edges, are
   * what has to fit between DC and Nyquist. */
  if (!(pass_low - transition > 0.0)) {
    return FIR_BANDPASS_ERR_BAND;
  }
  if (!(pass_high + transition < 0.5 * sample_rate)) {
    return FIR_BANDPASS_ERR_BAND;
  }

  /* The margin goes into both the window and the order: see the header. */
  const double design_db = attenuation + FIR_BANDPASS_DESIGN_MARGIN_DB;
  if (kaiser_taps(design_db, transition, sample_rate) > (size_t)FIR_BANDPASS_TAPS) {
    return FIR_BANDPASS_ERR_TAPS;
  }

  /* Every available tap is used, not just the estimated minimum. Spare taps
   * buy a narrower transition than was asked for, which is never a problem,
   * and a fixed-length convolution loop, which the compiler prefers. */
  const double beta = kaiser_beta(design_db);
  const double i0_beta = bessel_i0(beta);
  const double cut_low = pass_low - 0.5 * transition;   /* -6 dB points, mid-skirt */
  const double cut_high = pass_high + 0.5 * transition;
  const double centre = 0.5 * (double)(FIR_BANDPASS_TAPS - 1);

  for (size_t n = 0u; n < (size_t)FIR_BANDPASS_HALF_TAPS; n++) {
    const double offset = (double)n - centre; /* whole samples: taps are odd */

    /* Ideal bandpass: a lowpass at the upper cutoff less one at the lower. */
    const double ideal = (2.0 * cut_high / sample_rate) * sinc_pi(2.0 * cut_high * offset / sample_rate)
                       - (2.0 * cut_low / sample_rate) * sinc_pi(2.0 * cut_low * offset / sample_rate);

    /* Kaiser window, +-1 at the ends and 0 at the centre. The clamp is for
     * the rounding at the very first tap, where the radicand can go a hair
     * below zero. */
    const double position = offset / centre;
    double radicand = 1.0 - position * position;
    if (radicand < 0.0) {
      radicand = 0.0;
    }
    const double window = bessel_i0(beta * sqrt(radicand)) / i0_beta;

    filter->coeff[n] = (float)(ideal * window);
  }

  /* Normalise to unity in the passband. The design is symmetric, so its
   * amplitude response is real and this is just its value at band centre:
   * h[c] + 2 * sum of h[k] cos(omega (k - c)). Accumulated in double because
   * the sum matters more than the individual terms. */
  const double centre_rad = 2.0 * FIR_PI * (0.5 * (cut_low + cut_high)) / sample_rate;
  double gain = (double)filter->coeff[FIR_BANDPASS_HALF_TAPS - 1u];
  for (size_t k = 0u; k + 1u < (size_t)FIR_BANDPASS_HALF_TAPS; k++) {
    gain += 2.0 * (double)filter->coeff[k] * cos(centre_rad * ((double)k - centre));
  }
  if (!(fabs(gain) > 1.0e-9)) {
    memset(filter, 0, sizeof *filter); /* no usable passband; do not divide by it */
    return FIR_BANDPASS_ERR_GAIN;
  }
  for (size_t k = 0u; k < (size_t)FIR_BANDPASS_HALF_TAPS; k++) {
    filter->coeff[k] = (float)((double)filter->coeff[k] / gain);
  }

  filter->spec = *spec;
  return FIR_BANDPASS_OK;
}

/* ------------------------------------------------------------- audio path */

void fir_bandpass_reset(fir_bandpass_t* filter) {
  memset(filter->history, 0, sizeof filter->history);
  filter->write = 0u;
}

float fir_bandpass_tick(fir_bandpass_t* filter, float sample) {
  /* Stored twice, one tap count apart, so that the window below is contiguous
   * wherever the write position happens to be. */
  filter->history[filter->write] = sample;
  filter->history[filter->write + (size_t)FIR_BANDPASS_TAPS] = sample;
  filter->write = (filter->write + 1u < (size_t)FIR_BANDPASS_TAPS) ? filter->write + 1u : 0u;

  /* After the advance, history[write] is the oldest sample still in the
   * window and the next FIR_BANDPASS_TAPS entries run forward to the newest. */
  const float* oldest = &filter->history[filter->write];
  const float* newest = oldest + (FIR_BANDPASS_TAPS - 1);

  /* Symmetry: tap k and tap N-1-k share a coefficient, so add the two samples
   * first and multiply once. Half the multiplies of the direct form. */
  float accumulator = 0.0f;
  for (size_t k = 0u; k + 1u < (size_t)FIR_BANDPASS_HALF_TAPS; k++) {
    accumulator += filter->coeff[k] * (*oldest + *newest);
    oldest++;
    newest--;
  }
  /* The centre tap is its own mirror image, so it is counted once. */
  return accumulator + (filter->coeff[FIR_BANDPASS_HALF_TAPS - 1u] * (*oldest));
}

void fir_bandpass_process(fir_bandpass_t* filter, const float* in, float* out,
                          size_t count) {
  /* in[i] is read before out[i] is written, which is what makes aliasing safe. */
  for (size_t i = 0u; i < count; i++) {
    out[i] = fir_bandpass_tick(filter, in[i]);
  }
}

/* ----------------------------------------------------------- verification */

float fir_bandpass_response_db(const fir_bandpass_t* filter, float hz) {
  if (!((double)filter->spec.sample_rate_hz > 0.0)) {
    return -240.0f; /* never initialised */
  }

  const double omega = 2.0 * FIR_PI * (double)hz / (double)filter->spec.sample_rate_hz;
  const double centre = 0.5 * (double)(FIR_BANDPASS_TAPS - 1);

  /* Same folded sum as the normalisation in init, and real for the same
   * reason: a symmetric FIR has linear phase, so all the magnitude
   * information is in one real amplitude term. */
  double amplitude = (double)filter->coeff[FIR_BANDPASS_HALF_TAPS - 1u];
  for (size_t k = 0u; k + 1u < (size_t)FIR_BANDPASS_HALF_TAPS; k++) {
    amplitude += 2.0 * (double)filter->coeff[k] * cos(omega * ((double)k - centre));
  }

  const double magnitude = fabs(amplitude);
  if (!(magnitude > 1.0e-12)) {
    return -240.0f;
  }
  return (float)(20.0 * log10(magnitude));
}
