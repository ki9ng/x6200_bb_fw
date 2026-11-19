/*
 * Host-side checks of three pure DSP blocks in App/SDR, using the generated
 * coefficient tables the firmware actually ships:
 *
 *   sdr_halfband_decim.c  - output must match a plain CMSIS FIR decimator
 *                           (arm_fir_decimate_f32) on the same taps, and meet
 *                           the passband/alias numbers in the table header.
 *   sdr_iir_decim.c       - elliptic decimator meets its passband/stopband.
 *   sdr_soft_clipper.c    - continuous and slope-1 at the threshold, monotonic,
 *                           never reaches the ceiling, neg_only leaves +ve alone.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "arm_math.h"
#include "sdr_halfband_decim.h"
#include "sdr_iir_decim.h"
#include "sdr_soft_clipper.h"
#include "sdr_resampling_filters.h"

#define BLOCK 256u
#define NBLK  64u
#define SKIP  8u            /* blocks to skip while filters settle */

static int failures;

static void expect(const char *what, double got, double lo, double hi)
{
    int ok = got >= lo && got <= hi;
    if (!ok) failures++;
    printf("  %-44s %10.4g  [%s, want %g..%g]\n", what, got,
           ok ? "ok" : "FAIL", lo, hi);
}

static double db(double out_pow, double in_pow)
{
    return 10.0 * log10(out_pow / in_pow + 1e-30);
}

/* ---- half-band decimator -------------------------------------------- */

static void halfband_case(const char *name, const float32_t *coeffs,
                          uint32_t taps, double fs, double f_pass, double f_stop)
{
    printf("%s (%u taps, %.0f kHz in)\n", name, taps, fs / 1000.0);

    SdrHalfbandDecim hb;
    static float32_t hb_state[128 + BLOCK];
    memset(hb_state, 0, sizeof hb_state);
    Sdr_HalfbandDecim_Init(&hb, coeffs, taps, hb_state, BLOCK);

    arm_fir_decimate_instance_f32 ref;
    static float32_t ref_state[128 + BLOCK];
    memset(ref_state, 0, sizeof ref_state);
    arm_fir_decimate_init_f32(&ref, (uint16_t)taps, 2, coeffs, ref_state, BLOCK);

    /* 1. Equivalence on a wideband test signal (sum of tones). */
    float32_t in[BLOCK], a[BLOCK / 2], b[BLOCK / 2];
    double max_err = 0.0;
    for (uint32_t blk = 0; blk < NBLK; blk++) {
        for (uint32_t k = 0; k < BLOCK; k++) {
            double t = (double)(blk * BLOCK + k) / fs;
            in[k] = (float)(0.3 * sin(2 * M_PI * 0.11 * fs * t) +
                            0.3 * sin(2 * M_PI * 0.37 * fs * t + 1.0) +
                            0.3 * sin(2 * M_PI * 0.03 * fs * t + 2.0));
        }
        Sdr_HalfbandDecim_Process(&hb, in, a, BLOCK);
        arm_fir_decimate_f32(&ref, in, b, BLOCK);
        for (uint32_t k = 0; k < BLOCK / 2; k++)
            if (fabs(a[k] - b[k]) > max_err) max_err = fabs(a[k] - b[k]);
    }
    expect("max |halfband - CMSIS FIR decimate|", max_err, 0.0, 1e-5);

    /* 2. Passband tone and a tone that would alias into the passband. */
    double tests[2] = { f_pass * 0.9, f_stop * 1.05 };
    for (int c = 0; c < 2; c++) {
        double f = tests[c];
        Sdr_HalfbandDecim_Reset(&hb);
        double pin = 0, pout = 0;
        for (uint32_t blk = 0; blk < NBLK; blk++) {
            for (uint32_t k = 0; k < BLOCK; k++)
                in[k] = (float)sin(2 * M_PI * f * (double)(blk * BLOCK + k) / fs);
            Sdr_HalfbandDecim_Process(&hb, in, a, BLOCK);
            if (blk < SKIP) continue;
            for (uint32_t k = 0; k < BLOCK; k++)  pin  += (double)in[k] * in[k];
            for (uint32_t k = 0; k < BLOCK / 2; k++) pout += (double)a[k] * a[k];
        }
        /* output has half the samples, so compare mean power */
        double g = db(pout / (BLOCK / 2), pin / BLOCK);
        char label[80];
        if (c == 0) {
            snprintf(label, sizeof label, "passband %.0f Hz gain (dB)", f);
            expect(label, g, -0.5, 0.5);
        } else {
            snprintf(label, sizeof label, "aliasing tone %.0f Hz gain (dB)", f);
            expect(label, g, -300, -50);
        }
    }
}

/* ---- IIR decimator --------------------------------------------------- */

static double iir_decim_gain(double fs, double f)
{
    SdrIirDecim d;
    float state[4 * SDR_DECIM_384_90_IIR_SECTIONS] = {0};
    static float scratch[BLOCK];
    Sdr_IirDecim_Init(&d, sdr_decim_384_90_iir_sos,
                      SDR_DECIM_384_90_IIR_SECTIONS, state, scratch);
    float in[BLOCK], out[BLOCK / 2];
    double pin = 0, pout = 0;
    for (uint32_t blk = 0; blk < NBLK; blk++) {
        for (uint32_t k = 0; k < BLOCK; k++)
            in[k] = (float)sin(2 * M_PI * f * (double)(blk * BLOCK + k) / fs);
        Sdr_IirDecim_Process(&d, in, out, BLOCK);
        if (blk < SKIP) continue;
        for (uint32_t k = 0; k < BLOCK; k++)  pin  += (double)in[k] * in[k];
        for (uint32_t k = 0; k < BLOCK / 2; k++) pout += (double)out[k] * out[k];
    }
    return db(pout / (BLOCK / 2), pin / BLOCK);
}

/* ---- soft clipper ---------------------------------------------------- */

static float clip1(const SdrSoftClipperConfig *c, float x)
{
    float y;
    Sdr_SoftClipper_Process(c, &x, &y, 1);
    return y;
}

static void clipper_case(const char *name, float T, float M, bool neg_only)
{
    printf("soft clipper %s (T=%.2f, M=%.2f%s)\n", name, T, M,
           neg_only ? ", neg_only" : "");
    SdrSoftClipperConfig c;
    int r = Sdr_SoftClipper_Setup(&c, T, M, neg_only);
    expect("setup result", r, 0, 0);

    expect("below threshold is untouched", clip1(&c, -0.5f * T) - (-0.5f * T), 0, 0);
    float eps = 1e-3f * (M - T);  /* knee width scales with M-T */
    expect("continuous at -T (jump)",
           fabsf(clip1(&c, -T - eps) - clip1(&c, -T + eps)), 0, 4 * eps);
    double slope = (clip1(&c, -T - eps) - clip1(&c, -T - 2 * eps)) / eps;
    expect("|slope| just past -T", fabs(slope), 0.99, 1.001);

    double worst = 0; int mono = 1; float prev = 0.0f;
    for (int i = 0; i <= 100000; i++) {
        float x = -100.0f * M * (float)i / 100000.0f;
        float y = clip1(&c, x);
        if (fabsf(y) > worst) worst = fabsf(y);
        if (i && y > prev + 1e-7f) mono = 0;
        prev = y;
    }
    expect("max |output| for inputs to 100*M", worst, 0, M);
    expect("monotonic (1 = yes)", mono, 1, 1);
    if (neg_only)
        expect("positive input passes unchanged", clip1(&c, 3.0f * M) - 3.0f * M, 0, 0);
    else
        expect("odd symmetry y(x)+y(-x)", clip1(&c, 2.5f * M) + clip1(&c, -2.5f * M),
               -1e-6, 1e-6);
}

int main(void)
{
    halfband_case("decim_48_9",  sdr_decim_48_9_coeffs,  SDR_DECIM_48_9_NUM_TAPS,
                  48000.0, 9000.0, 15000.0);
    halfband_case("decim_24_5",  sdr_decim_24_5_coeffs,  SDR_DECIM_24_5_NUM_TAPS,
                  24000.0, 5000.0, 7000.0);
    halfband_case("decim_96_15", sdr_decim_96_15_coeffs, SDR_DECIM_96_15_NUM_TAPS,
                  96000.0, 15000.0, 33000.0);

    printf("decim_384_90 IIR (384 kHz in)\n");
    expect("passband 80 kHz gain (dB)", iir_decim_gain(384000.0, 80000.0), -0.5, 0.5);
    expect("stopband 110 kHz gain (dB)", iir_decim_gain(384000.0, 110000.0), -300, -65);

    clipper_case("voice", 2.0f, 3.0f, false);   /* as set up in sdr_tx.c */
    clipper_case("AM",    0.95f, 1.0f, true);

    printf(failures ? "FAILED: %d\n" : "all ok\n", failures);
    return failures ? 1 : 0;
}
