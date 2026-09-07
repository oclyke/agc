/**
 * @file test_fir_bandpass.c
 * @brief Native tests for the FIR bandpass library.
 *
 * Build and run:   make test
 *
 * The point of these is that they MEASURE rather than assume. The design
 * follows closed-form estimates that are known to be approximate, so nothing
 * here trusts them: the response is swept across the whole spectrum and
 * checked against the spec, with a reference DFT written independently of the
 * folded sum the library uses, and the streaming path is checked against a
 * direct convolution so a bug in the history buffer cannot hide behind a
 * correct set of coefficients.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fir_bandpass.h"

#define TEST_PI (3.14159265358979323846)

/* The project's filter: 8 kHz, 300 Hz to 3000 Hz, 150 Hz skirts, 60 dB down. */
#define SPEC_RATE (8000.0f)
#define SPEC_LOW (300.0f)
#define SPEC_HIGH (3000.0f)
#define SPEC_SKIRT (150.0f)
#define SPEC_STOP_DB (60.0f)

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

static fir_bandpass_spec_t project_spec(void) {
  fir_bandpass_spec_t spec;
  spec.sample_rate_hz = SPEC_RATE;
  spec.pass_low_hz = SPEC_LOW;
  spec.pass_high_hz = SPEC_HIGH;
  spec.transition_hz = SPEC_SKIRT;
  spec.stopband_db = SPEC_STOP_DB;
  return spec;
}

/* One filter, not one per test: it is 2 KB and the tests do not modify the
 * coefficients, only the history, which they reset. */
static fir_bandpass_t filter;
static float impulse_response[FIR_BANDPASS_TAPS];

/**
 * @brief Reference magnitude response, straight from the definition.
 *
 * Sums the full complex DFT over every tap. The library folds the sum in half
 * and drops the imaginary part, which is valid only because the coefficients
 * are symmetric - so this deliberately does neither, and disagreement between
 * the two means the symmetry assumption has broken.
 */
static double reference_response_db(const float* taps, size_t count, double hz,
                                    double sample_rate) {
  double real = 0.0;
  double imag = 0.0;
  for (size_t n = 0u; n < count; n++) {
    const double angle = 2.0 * TEST_PI * hz * (double)n / sample_rate;
    real += (double)taps[n] * cos(angle);
    imag -= (double)taps[n] * sin(angle);
  }
  const double magnitude = sqrt(real * real + imag * imag);
  return 20.0 * log10(magnitude > 1.0e-300 ? magnitude : 1.0e-300);
}

/** Pull the impulse response out through the public API. */
static void capture_impulse_response(void) {
  fir_bandpass_reset(&filter);
  impulse_response[0] = fir_bandpass_tick(&filter, 1.0f);
  for (size_t n = 1u; n < (size_t)FIR_BANDPASS_TAPS; n++) {
    impulse_response[n] = fir_bandpass_tick(&filter, 0.0f);
  }
  fir_bandpass_reset(&filter);
}

static void test_sizing(void) {
  printf("Tap count\n");

  fir_bandpass_spec_t spec = project_spec();
  const size_t needed = fir_bandpass_design_taps(&spec);
  printf("        the project spec needs %zu taps; this build has %d\n", needed,
         FIR_BANDPASS_TAPS);
  CHECK(needed == (size_t)FIR_BANDPASS_TAPS,
        "the default tap count is exactly what the project spec needs");
  CHECK((needed % 2u) == 1u, "tap count is odd");

  /* Halving the skirt roughly doubles the taps, which this build cannot hold. */
  spec.transition_hz = 50.0f;
  CHECK(fir_bandpass_design_taps(&spec) > (size_t)FIR_BANDPASS_TAPS,
        "a 50 Hz skirt is reported as needing more taps than this build has");
  CHECK(fir_bandpass_init(&filter, &spec) == FIR_BANDPASS_ERR_TAPS,
        "and init refuses it rather than under-delivering attenuation");

  /* A wider skirt fits with room to spare, and is allowed to use every tap.
   * 250 Hz is as wide as this passband can take: any more and the lower skirt
   * would run past DC, which is a different complaint entirely. */
  spec.transition_hz = 250.0f;
  CHECK(fir_bandpass_design_taps(&spec) < (size_t)FIR_BANDPASS_TAPS,
        "a 250 Hz skirt needs fewer taps than this build has");
  CHECK(fir_bandpass_init(&filter, &spec) == FIR_BANDPASS_OK, "and init accepts it");

  CHECK(fir_bandpass_design_taps(NULL) == 0u, "design_taps rejects NULL");
}

static void test_spec_validation(void) {
  printf("Spec validation\n");

  fir_bandpass_spec_t spec = project_spec();
  CHECK(fir_bandpass_init(NULL, &spec) == FIR_BANDPASS_ERR_NULL, "NULL filter");
  CHECK(fir_bandpass_init(&filter, NULL) == FIR_BANDPASS_ERR_NULL, "NULL spec");

  spec = project_spec();
  spec.sample_rate_hz = 0.0f;
  CHECK(fir_bandpass_init(&filter, &spec) == FIR_BANDPASS_ERR_RATE, "zero sample rate");

  spec = project_spec();
  spec.transition_hz = -1.0f;
  CHECK(fir_bandpass_init(&filter, &spec) == FIR_BANDPASS_ERR_TRANSITION,
        "negative skirt width");

  spec = project_spec();
  spec.stopband_db = 5.0f;
  CHECK(fir_bandpass_init(&filter, &spec) == FIR_BANDPASS_ERR_ATTENUATION,
        "attenuation below the useful range");

  spec = project_spec();
  spec.pass_low_hz = 3500.0f;
  CHECK(fir_bandpass_init(&filter, &spec) == FIR_BANDPASS_ERR_BAND,
        "passband edges out of order");

  spec = project_spec();
  spec.pass_low_hz = 100.0f; /* skirt would run past DC */
  CHECK(fir_bandpass_init(&filter, &spec) == FIR_BANDPASS_ERR_BAND,
        "lower skirt running past DC");

  spec = project_spec();
  spec.pass_high_hz = 3900.0f; /* skirt would run past Nyquist */
  CHECK(fir_bandpass_init(&filter, &spec) == FIR_BANDPASS_ERR_BAND,
        "upper skirt running past Nyquist");

  spec = project_spec();
  spec.sample_rate_hz = (float)NAN;
  CHECK(fir_bandpass_init(&filter, &spec) == FIR_BANDPASS_ERR_RATE,
        "a NaN sample rate is rejected, not propagated");

  /* A rejected init must leave silence behind, not a half-built design. */
  CHECK(fir_bandpass_tick(&filter, 1.0f) == 0.0f,
        "a filter left by a failed init outputs silence");
}

static void test_frequency_response(void) {
  printf("Frequency response against the spec\n");

  const fir_bandpass_spec_t spec = project_spec();
  CHECK(fir_bandpass_init(&filter, &spec) == FIR_BANDPASS_OK, "the project spec designs");
  capture_impulse_response();

  const double nyquist = SPEC_RATE / 2.0;
  const double stop_low = SPEC_LOW - SPEC_SKIRT;   /* 150 Hz  */
  const double stop_high = SPEC_HIGH + SPEC_SKIRT; /* 3150 Hz */

  double pass_max = -1000.0, pass_min = 1000.0, stop_max = -1000.0;
  double pass_worst_hz = 0.0, stop_worst_hz = 0.0;
  double reference_error = 0.0;

  /* 0.25 Hz steps: fine enough that no ripple lobe can hide between samples. */
  const long steps = (long)(nyquist * 4.0);
  for (long i = 0; i <= steps; i++) {
    const double hz = (double)i * 0.25;
    const double db = (double)fir_bandpass_response_db(&filter, (float)hz);

    const double reference = reference_response_db(impulse_response, FIR_BANDPASS_TAPS,
                                                   hz, SPEC_RATE);
    if (db > -200.0 && fabs(db - reference) > reference_error) {
      reference_error = fabs(db - reference);
    }

    if (hz >= SPEC_LOW && hz <= SPEC_HIGH) {
      if (db > pass_max) { pass_max = db; }
      if (db < pass_min) { pass_min = db; pass_worst_hz = hz; }
    }
    if (hz <= stop_low || hz >= stop_high) {
      if (db > stop_max) { stop_max = db; stop_worst_hz = hz; }
    }
  }

  printf("        passband %.0f-%.0f Hz : %+.3f .. %+.3f dB (ripple %.3f dB)\n",
         SPEC_LOW, SPEC_HIGH, pass_min, pass_max, pass_max - pass_min);
  printf("        stopbands <=%.0f, >=%.0f Hz : worst %+.2f dB at %.0f Hz\n",
         stop_low, stop_high, stop_max, stop_worst_hz);
  printf("        folded response vs reference DFT : max %.2e dB apart\n",
         reference_error);
  (void)pass_worst_hz;

  CHECK(stop_max <= -(double)SPEC_STOP_DB,
        "stopbands are at least 60 dB down, everywhere");
  CHECK(pass_max - pass_min < 0.1, "passband is flat within 0.1 dB");
  CHECK(fabs(pass_max) < 0.1, "passband gain is unity within 0.1 dB");
  CHECK(reference_error < 1.0e-3,
        "folded response agrees with an independent DFT of the impulse response");

  /* The -6 dB points should land in the middle of each skirt. */
  const double low_cut = (double)fir_bandpass_response_db(&filter, SPEC_LOW - SPEC_SKIRT / 2.0f);
  const double high_cut = (double)fir_bandpass_response_db(&filter, SPEC_HIGH + SPEC_SKIRT / 2.0f);
  printf("        -6 dB points : %.2f dB at 225 Hz, %.2f dB at 3075 Hz\n", low_cut, high_cut);
  CHECK(fabs(low_cut + 6.02) < 0.3, "lower cutoff sits mid-skirt at 225 Hz");
  CHECK(fabs(high_cut + 6.02) < 0.3, "upper cutoff sits mid-skirt at 3075 Hz");

  /* Spot checks either side of the band, including DC and Nyquist. */
  CHECK(fir_bandpass_response_db(&filter, 0.0f) <= -(float)SPEC_STOP_DB, "DC is rejected");
  CHECK(fir_bandpass_response_db(&filter, (float)nyquist) <= -(float)SPEC_STOP_DB,
        "Nyquist is rejected");
  CHECK(fir_bandpass_response_db(&filter, 50.0f) <= -(float)SPEC_STOP_DB,
        "50 Hz mains hum is rejected");
  CHECK(fir_bandpass_response_db(&filter, 1000.0f) > -0.1f, "1 kHz passes");
}

static void test_impulse_response(void) {
  printf("Impulse response\n");

  capture_impulse_response();

  /* Exactly equal, not nearly: the convolution of an impulse adds a run of
   * zeros to one coefficient, and adding 0.0f changes no float. If this ever
   * fails the fold has gone wrong, not the arithmetic. */
  int symmetric = 1;
  for (size_t n = 0u; n < (size_t)FIR_BANDPASS_TAPS; n++) {
    if (impulse_response[n] != impulse_response[FIR_BANDPASS_TAPS - 1u - n]) {
      symmetric = 0;
    }
  }
  CHECK(symmetric, "impulse response is symmetric, so the phase is linear");

  size_t peak = 0u;
  for (size_t n = 1u; n < (size_t)FIR_BANDPASS_TAPS; n++) {
    if (fabsf(impulse_response[n]) > fabsf(impulse_response[peak])) {
      peak = n;
    }
  }
  printf("        peak tap %zu of %d, value %.6f; group delay %d samples (%.2f ms)\n",
         peak, FIR_BANDPASS_TAPS, (double)impulse_response[peak],
         FIR_BANDPASS_GROUP_DELAY, 1000.0 * FIR_BANDPASS_GROUP_DELAY / SPEC_RATE);
  CHECK(peak == (size_t)FIR_BANDPASS_GROUP_DELAY,
        "the peak sits at the advertised group delay");

  /* The centre tap of a bandpass is twice the fractional bandwidth. */
  const double expected_centre = 2.0 * ((SPEC_HIGH + SPEC_SKIRT / 2.0) -
                                        (SPEC_LOW - SPEC_SKIRT / 2.0)) / SPEC_RATE;
  CHECK(fabs((double)impulse_response[FIR_BANDPASS_GROUP_DELAY] - expected_centre) < 0.01,
        "the centre tap matches the fractional bandwidth");

  /* The tick path should reproduce the designed coefficients exactly. */
  int matches_design = 1;
  for (size_t n = 0u; n < (size_t)FIR_BANDPASS_HALF_TAPS; n++) {
    if (impulse_response[n] != filter.coeff[n]) {
      matches_design = 0;
    }
  }
  CHECK(matches_design, "the streaming path reproduces the designed coefficients");
}

static void test_streaming(void) {
  printf("Streaming path\n");

  enum { SAMPLES = 1000 };
  static float input[SAMPLES];
  static float streamed[SAMPLES];
  static float blocked[SAMPLES];
  static float in_place[SAMPLES];

  /* Deterministic pseudorandom input, so a failure is reproducible. */
  unsigned long seed = 12345u;
  for (size_t n = 0u; n < (size_t)SAMPLES; n++) {
    seed = (seed * 1103515245u + 12345u) & 0x7FFFFFFFu;
    input[n] = ((float)seed / 1073741824.0f) - 1.0f;
  }

  capture_impulse_response();

  fir_bandpass_reset(&filter);
  for (size_t n = 0u; n < (size_t)SAMPLES; n++) {
    streamed[n] = fir_bandpass_tick(&filter, input[n]);
  }

  /* Direct convolution, the definition, with no ring buffer anywhere near it.
   * 1000 samples is well past the 217-entry history, so the wrap is covered. */
  double worst = 0.0;
  for (size_t n = 0u; n < (size_t)SAMPLES; n++) {
    double expected = 0.0;
    for (size_t k = 0u; k < (size_t)FIR_BANDPASS_TAPS; k++) {
      if (k <= n) {
        expected += (double)impulse_response[k] * (double)input[n - k];
      }
    }
    const double error = fabs(expected - (double)streamed[n]);
    if (error > worst) { worst = error; }
  }
  printf("        streaming vs direct convolution over %d samples : worst %.2e\n",
         SAMPLES, worst);
  CHECK(worst < 1.0e-5, "streaming matches a direct convolution across the wrap");

  /* Blocks of 64, the size the capture path delivers, must give the same
   * answer as one sample at a time. */
  fir_bandpass_reset(&filter);
  for (size_t offset = 0u; offset < (size_t)SAMPLES; offset += 64u) {
    size_t count = 64u;
    if (offset + count > (size_t)SAMPLES) { count = (size_t)SAMPLES - offset; }
    fir_bandpass_process(&filter, &input[offset], &blocked[offset], count);
  }
  CHECK(memcmp(streamed, blocked, sizeof streamed) == 0,
        "64-sample blocks give the same result as sample-at-a-time");

  /* Filtering in place must give the same answer as filtering out of place. */
  memcpy(in_place, input, sizeof input);
  fir_bandpass_reset(&filter);
  fir_bandpass_process(&filter, in_place, in_place, (size_t)SAMPLES);
  CHECK(memcmp(streamed, in_place, sizeof streamed) == 0,
        "in-place processing matches out-of-place");

  /* Reset must actually clear the history, or the next block starts with the
   * tail of the last one convolved into it. */
  fir_bandpass_reset(&filter);
  float first = 0.0f;
  for (size_t n = 0u; n < (size_t)FIR_BANDPASS_TAPS; n++) {
    first = fir_bandpass_tick(&filter, 0.0f);
  }
  CHECK(first == 0.0f, "reset clears the history");
}

static void test_tones(void) {
  printf("Tones through the filter\n");

  const fir_bandpass_spec_t spec = project_spec();
  CHECK(fir_bandpass_init(&filter, &spec) == FIR_BANDPASS_OK, "designed");

  static const double tones[] = {50.0, 100.0, 150.0, 300.0, 1000.0, 2000.0,
                                 3000.0, 3150.0, 3600.0, 3950.0};
  const size_t tone_count = sizeof tones / sizeof tones[0];

  for (size_t t = 0u; t < tone_count; t++) {
    const double hz = tones[t];
    fir_bandpass_reset(&filter);

    /* Settle for the length of the filter, then measure over 2000 samples. */
    double energy = 0.0;
    const long settle = FIR_BANDPASS_TAPS;
    const long measure = 2000;
    for (long n = 0; n < settle + measure; n++) {
      const float x = (float)sin(2.0 * TEST_PI * hz * (double)n / SPEC_RATE);
      const float y = fir_bandpass_tick(&filter, x);
      if (n >= settle) { energy += (double)y * (double)y; }
    }
    /* A unit sine has RMS 1/sqrt(2), so this is the gain. */
    const double gain_db = 20.0 * log10(sqrt(2.0 * energy / (double)measure) + 1.0e-300);
    const double predicted = (double)fir_bandpass_response_db(&filter, (float)hz);

    const int in_band = (hz >= SPEC_LOW && hz <= SPEC_HIGH);
    const int in_stop = (hz <= SPEC_LOW - SPEC_SKIRT || hz >= SPEC_HIGH + SPEC_SKIRT);
    const char* verdict = in_band ? "pass" : (in_stop ? "stop" : "skirt");

    printf("        %6.0f Hz  %5s : measured %+8.2f dB, predicted %+8.2f dB\n", hz,
           verdict, gain_db, predicted);

    if (in_band) {
      CHECK(fabs(gain_db) < 0.2, "in-band tone passes at unity gain");
    }
    if (in_stop) {
      CHECK(gain_db <= -(double)SPEC_STOP_DB, "out-of-band tone is 60 dB down");
    }
    CHECK(fabs(gain_db - predicted) < 0.5,
          "measured gain agrees with the predicted response");
  }
}

static void test_other_bands(void) {
  printf("Other bands in the same build\n");

  /* The tap count is fixed but the band is not: the same object, re-specified
   * at run time, and the spare taps go into a narrower skirt than asked for. */
  fir_bandpass_spec_t spec;
  spec.sample_rate_hz = 8000.0f;
  spec.pass_low_hz = 800.0f;
  spec.pass_high_hz = 1200.0f;
  spec.transition_hz = 200.0f;
  spec.stopband_db = 60.0f;

  CHECK(fir_bandpass_init(&filter, &spec) == FIR_BANDPASS_OK,
        "a 800-1200 Hz band designs in the same build");
  CHECK(fir_bandpass_response_db(&filter, 1000.0f) > -0.1f, "1 kHz passes");
  CHECK(fir_bandpass_response_db(&filter, 400.0f) <= -60.0f, "400 Hz is rejected");
  CHECK(fir_bandpass_response_db(&filter, 1600.0f) <= -60.0f, "1600 Hz is rejected");
  CHECK(filter.spec.pass_low_hz == 800.0f && filter.spec.pass_high_hz == 1200.0f,
        "the filter carries the spec it was built for");

  /* A different sample rate entirely. Twice the rate needs twice the skirt to
   * fit in the same taps, because what costs taps is the skirt as a fraction
   * of the sample rate, not its width in Hz. */
  spec.sample_rate_hz = 16000.0f;
  spec.pass_low_hz = 400.0f;
  spec.pass_high_hz = 3400.0f;
  spec.transition_hz = 300.0f;
  CHECK(fir_bandpass_init(&filter, &spec) == FIR_BANDPASS_OK, "and so does a 16 kHz band");
  CHECK(fir_bandpass_response_db(&filter, 1000.0f) > -0.1f, "1 kHz passes at 16 kHz");
  CHECK(fir_bandpass_response_db(&filter, 6000.0f) <= -60.0f, "6 kHz is rejected at 16 kHz");
}

int main(void) {
  printf("FIR bandpass, %d taps, %zu bytes per instance\n\n", FIR_BANDPASS_TAPS,
         sizeof(fir_bandpass_t));

  test_sizing();
  test_spec_validation();
  test_frequency_response();
  test_impulse_response();
  test_streaming();
  test_tones();
  test_other_bands();

  printf("\n%s\n", failures == 0 ? "ALL FIR TESTS PASSED" : "SOME FIR TESTS FAILED");
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
