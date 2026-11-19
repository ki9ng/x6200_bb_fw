/*
 * Host-side check of App/SDR/sdr_iir.c: design the SSB RX passband filters the
 * same way sdr_ssb.c does (elliptic, 6 biquad stages, NOISE_BW edge), run them
 * through the real CMSIS-DSP df1 cascade, and measure the gain of pure tones.
 *
 * No hardware: sdr_iir.c is pure math, and CMSIS-DSP's portable C path builds
 * on x86. Prints a response table and fails if passband/stopband limits break.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "arm_math.h"
#include "sdr_iir.h"

#define FS        12000.0f   /* assumed SSB RX rate; see Makefile note */
#define N_SAMPLES 24000u     /* 2 s per tone */
#define SETTLE    6000u      /* skip the filter's start-up transient */
#define BLOCK     240u

static int failures;

static float tone_gain_db(const float32_t *coeffs, uint8_t stages, float f_hz)
{
    float32_t state[4 * 8] = {0};
    arm_biquad_casd_df1_inst_f32 inst;
    arm_biquad_cascade_df1_init_f32(&inst, stages, coeffs, state);

    double in_pow = 0.0, out_pow = 0.0;
    float32_t in[BLOCK], out[BLOCK];
    for (uint32_t n0 = 0; n0 < N_SAMPLES; n0 += BLOCK) {
        for (uint32_t k = 0; k < BLOCK; k++)
            in[k] = sinf(2.0f * (float)M_PI * f_hz * (float)(n0 + k) / FS);
        arm_biquad_cascade_df1_f32(&inst, in, out, BLOCK);
        if (n0 >= SETTLE)
            for (uint32_t k = 0; k < BLOCK; k++) {
                in_pow  += (double)in[k] * in[k];
                out_pow += (double)out[k] * out[k];
            }
    }
    return (float)(10.0 * log10(out_pow / in_pow + 1e-30));
}

static void expect(const char *what, float got, float lo, float hi)
{
    int ok = got >= lo && got <= hi;
    if (!ok) failures++;
    printf("  %-26s %8.2f dB  [%s, want %.1f..%.1f]\n", what, got,
           ok ? "ok" : "FAIL", lo, hi);
}

static void check(SdrIirType type, float edge_hz)
{
    SdrIirSpec spec = {
        .type = type, .edge = SDR_IIR_EDGE_NOISE_BW,
        .stages = SDR_IIR_STAGES_6, .fs_hz = FS, .f_edge_hz = edge_hz,
    };
    float32_t coeffs[SDR_IIR_COEFFS_PER_STAGE * 8];
    uint8_t stages = 0;
    SdrIirResponse r;
    int res = Sdr_Iir_DesignEllipEx(&spec, coeffs, &stages, &r);
    printf("%s edge %.0f Hz: res %d, %u stages, ref %.1f stop %.1f nbw %.1f\n",
           type == SDR_IIR_TYPE_LPF ? "LPF" : "HPF", edge_hz, res, stages,
           r.f_ref_hz, r.f_stop_hz, r.noise_bw_hz);
    if (res != SDR_IIR_OK) { failures++; return; }

    char label[64];
    float pf = type == SDR_IIR_TYPE_LPF ? r.f_ref_hz * 0.5f : r.f_ref_hz * 2.0f;
    float sf = type == SDR_IIR_TYPE_LPF ? r.f_stop_hz * 1.1f : r.f_stop_hz * 0.9f;
    snprintf(label, sizeof label, "passband %.0f Hz", pf);
    expect(label, tone_gain_db(coeffs, stages, pf), -1.5f, 1.0f);
    snprintf(label, sizeof label, "stopband %.0f Hz", sf);
    expect(label, tone_gain_db(coeffs, stages, sf), -200.0f, -40.0f);
}

int main(void)
{
    check(SDR_IIR_TYPE_LPF, 2700.0f);
    check(SDR_IIR_TYPE_LPF, 600.0f);
    check(SDR_IIR_TYPE_HPF, 300.0f);
    check(SDR_IIR_TYPE_HPF, 100.0f);
    printf(failures ? "FAILED: %d\n" : "all ok\n", failures);
    return failures ? 1 : 0;
}
