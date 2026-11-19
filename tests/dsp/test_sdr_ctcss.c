/*
 * Host-side check of App/SDR/sdr_ctcss.c (CTCSS tone encoder for NBFM TX) at
 * the NBFM modulator rate, 48 kHz.
 *
 * Checks every EIA/TIA tone plus the NATO 150.0 Hz tone for frequency, level
 * and purity over 10 s, then the transitions: zero-phase start,
 * phase-continuous frequency change, stop at a zero crossing, and range
 * rejection.
 *
 * The last section is information, not pass/fail: the TX voice HPF (same
 * design call as sdr_tx.c) at CTCSS and low voice frequencies, for the fixed
 * 150 Hz edge the firmware uses today and for a 300 Hz edge. The tone is
 * added after that HPF, so the HPF only decides how much voice lands in the
 * tone band.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "sdr_ctcss.h"
#include "sdr_iir.h"

#define FS     48000u
#define BLK    64u
#define SECS   10u
#define NSAMP  (FS * SECS)

static int failures;

static void expect(const char *what, double got, double lo, double hi)
{
    int ok = got >= lo && got <= hi;
    if (!ok) failures++;
    printf("  %-44s %10.4f  [%s, want %g..%g]\n", what, got,
           ok ? "ok" : "FAIL", lo, hi);
}

/* EIA/TIA-603 tones in 0.1 Hz, plus 150.0 (NATO). */
static const uint16_t k_tones[] = {
    670, 693, 719, 744, 770, 797, 825, 854, 885, 915, 948, 974, 1000, 1035,
    1072, 1109, 1148, 1188, 1230, 1273, 1318, 1365, 1413, 1462, 1500, 1514,
    1567, 1598, 1622, 1655, 1679, 1713, 1738, 1773, 1799, 1835, 1862, 1899,
    1928, 1966, 1995, 2035, 2065, 2107, 2181, 2257, 2291, 2336, 2418, 2503,
    2541,
};

static float s_buf[NSAMP];

static void run(uint32_t n, float *out)
{
    for (uint32_t i = 0; i < n; i += BLK) {
        uint32_t m = (n - i < BLK) ? n - i : BLK;
        if (!Sdr_Ctcss_Fill(out + i, m)) {
            for (uint32_t k = 0; k < m; k++) out[i + k] = 0.0f;
        }
    }
}

/* Frequency from rising zero crossings, linearly interpolated. */
static double zc_freq(const float *x, uint32_t n)
{
    double first = -1, last = -1;
    long count = 0;
    for (uint32_t i = 1; i < n; i++) {
        if (x[i - 1] < 0.0f && x[i] >= 0.0f) {
            double t = (i - 1) + x[i - 1] / (double)(x[i - 1] - x[i]);
            if (first < 0) first = t;
            last = t;
            count++;
        }
    }
    return (count > 1) ? (count - 1) * (double)FS / (last - first) : 0.0;
}

/* Hann-windowed single-bin DFT, returns the amplitude of a sine at f. */
static double tone_amp(const float *x, uint32_t n, double f)
{
    double re = 0, im = 0, wsum = 0;
    for (uint32_t i = 0; i < n; i++) {
        double w = 0.5 - 0.5 * cos(2 * M_PI * i / (n - 1));
        double ph = 2 * M_PI * f * i / FS;
        re += w * x[i] * cos(ph);
        im += w * x[i] * sin(ph);
        wsum += w;
    }
    return 2.0 * sqrt(re * re + im * im) / wsum;
}

static double max_step(const float *x, uint32_t a, uint32_t b)
{
    double m = 0;
    for (uint32_t i = a + 1; i < b; i++) {
        double d = fabs((double)x[i] - x[i - 1]);
        if (d > m) m = d;
    }
    return m;
}

static void check_tones(void)
{
    double worst_ferr = 0, worst_lvl = 0, worst_res = -300;
    uint16_t at_f = 0, at_l = 0, at_h = 0;

    for (unsigned t = 0; t < sizeof k_tones / sizeof k_tones[0]; t++) {
        double f = k_tones[t] / 10.0;
        Sdr_Ctcss_Init(FS);
        if (Sdr_Ctcss_Set(k_tones[t], 0) != 0) {
            failures++;
            printf("  Set(%u) rejected [FAIL]\n", k_tones[t]);
            continue;
        }
        run(NSAMP, s_buf);

        double ferr = fabs(zc_freq(s_buf, NSAMP) - f);
        double p = 0;
        for (uint32_t i = 0; i < NSAMP; i++) p += (double)s_buf[i] * s_buf[i];
        double peak = sqrt(2.0 * p / NSAMP);
        double lvl = fabs(peak / (SDR_CTCSS_DEV_DEFAULT_PERMILLE / 1000.0) - 1.0);
        /* Purity: everything that is not the ideal tone. The sine table's
         * interpolation error repeats every table step, so it lands near
         * 512 * f and folds back into the band, not at low harmonics; the
         * residual against the exact sine catches it wherever it falls. The
         * ideal uses the same rounded phase step, so this is not a
         * frequency check. */
        const double A = SDR_CTCSS_DEV_DEFAULT_PERMILLE / 1000.0;
        uint64_t den = (uint64_t)FS * 10u;
        uint32_t inc = (uint32_t)((((uint64_t)k_tones[t] << 32) + den / 2u) / den);
        double r = 0;
        for (uint32_t i = 0; i < NSAMP; i++) {
            uint32_t ph = inc * i;
            double e = s_buf[i] - A * sin(2 * M_PI * (ph / 4294967296.0));
            r += e * e;
        }
        double res = 10 * log10(r / NSAMP / (A * A / 2) + 1e-30);
        if (ferr > worst_ferr) { worst_ferr = ferr; at_f = k_tones[t]; }
        if (lvl > worst_lvl)   { worst_lvl = lvl;   at_l = k_tones[t]; }
        if (res > worst_res)   { worst_res = res;   at_h = k_tones[t]; }
    }
    printf("51 tones, 10 s each, default level (%u/1000 of peak deviation)\n",
           SDR_CTCSS_DEV_DEFAULT_PERMILLE);
    printf("  worst at: freq %.1f Hz, level %.1f Hz, purity %.1f Hz\n",
           at_f / 10.0, at_l / 10.0, at_h / 10.0);
    expect("worst frequency error, Hz", worst_ferr, 0, 0.001);
    expect("worst level error, % of nominal", 100 * worst_lvl, 0, 0.2);
    expect("worst residual (all non-tone content), dBc", worst_res, -200, -80);
}

static void check_transitions(void)
{
    const double A = SDR_CTCSS_DEV_DEFAULT_PERMILLE / 1000.0;
    printf("transitions (100.0 Hz unless noted)\n");

    /* Zero-phase start. */
    Sdr_Ctcss_Init(FS);
    float first[BLK];
    expect("Fill with no tone returns false", Sdr_Ctcss_Fill(first, BLK), 0, 0);
    Sdr_Ctcss_Set(1000, 0);
    Sdr_Ctcss_Fill(first, BLK);
    expect("first sample after enable", first[0], 0, 0);

    /* Phase-continuous change 100.0 -> 103.5 Hz at sample 24000. */
    Sdr_Ctcss_Init(FS);
    Sdr_Ctcss_Set(1000, 0);
    run(24000, s_buf);
    Sdr_Ctcss_Set(1035, 0);
    run(24000, s_buf + 24000);
    double bound = A * 2 * M_PI * 103.5 / FS;
    expect("max step across freq change / bound", max_step(s_buf, 23900, 24100) / bound, 0, 1.001);
    expect("frequency after change, Hz", zc_freq(s_buf + 25000, 23000), 103.499, 103.501);

    /* Reset (start of next TX) restarts at zero phase. */
    Sdr_Ctcss_Reset();
    Sdr_Ctcss_Fill(first, BLK);
    expect("first sample after Reset", first[0], 0, 0);

    /* Stop at the next zero crossing, requested near a peak. */
    Sdr_Ctcss_Init(FS);
    Sdr_Ctcss_Set(1000, 0);
    run(48000 + 120, s_buf);               /* 100 Hz: 120 samples = 1/4 cycle */
    Sdr_Ctcss_Set(0, 0);
    run(4800, s_buf + 48120);
    uint32_t last_nz = 48120;
    for (uint32_t i = 48120; i < 48120 + 4800; i++)
        if (s_buf[i] != 0.0f) last_nz = i;
    double stop_ms = (last_nz - 48120) * 1000.0 / FS;
    expect("tone ends after stop request, ms", stop_ms, 0, 5.01);
    expect("last sample before silence / one step", fabs(s_buf[last_nz]) / (A * 2 * M_PI * 100.0 / FS), 0, 1.001);
    expect("Fill after stop returns false", Sdr_Ctcss_Fill(first, BLK), 0, 0);

    /* Range checks, and a rejected Set leaves the running tone alone. */
    Sdr_Ctcss_Init(FS);
    Sdr_Ctcss_Set(885, 0);
    int r = 0;
    r += Sdr_Ctcss_Set(599, 0) == -1;
    r += Sdr_Ctcss_Set(2601, 0) == -1;
    r += Sdr_Ctcss_Set(1000, 251) == -1;
    expect("out-of-range Set calls rejected (of 3)", r, 3, 3);
    run(NSAMP / 5, s_buf);
    expect("tone still 88.5 Hz after rejects", zc_freq(s_buf, NSAMP / 5), 88.499, 88.501);

    /* Level follows dev_permille. */
    Sdr_Ctcss_Init(FS);
    Sdr_Ctcss_Set(1000, 150);
    run(NSAMP / 5, s_buf);
    expect("peak at 150/1000", tone_amp(s_buf, NSAMP / 5, 100.0), 0.1497, 0.1503);
}

/* Steady-state gain (dB) of the TX voice HPF at f, designed like sdr_tx.c. */
static double hpf_gain_db(uint32_t edge_hz, double f)
{
    SdrIirSpec spec = {
        .edge = SDR_IIR_EDGE_NOISE_BW,
        .f_edge_hz = (float)edge_hz,
        .fs_hz = (float)FS,
        .stages = SDR_IIR_STAGES_5,
        .type = SDR_IIR_TYPE_HPF,
    };
    float coeffs[5 * 5], state[5 * 4] = {0};
    uint8_t stages = 0;
    if (Sdr_Iir_DesignEllip(&spec, coeffs, &stages) != SDR_IIR_OK) return NAN;
    arm_biquad_casd_df1_inst_f32 inst;
    arm_biquad_cascade_df1_init_f32(&inst, 5, coeffs, state);

    const uint32_t n = 4 * FS;               /* settle 2 s, measure 2 s */
    float in[BLK], out[BLK];
    double p = 0;
    for (uint32_t i = 0; i < n; i += BLK) {
        for (uint32_t k = 0; k < BLK; k++)
            in[k] = (float)sin(2 * M_PI * f * (i + k) / FS);
        arm_biquad_cascade_df1_f32(&inst, in, out, BLK);
        if (i >= n / 2)
            for (uint32_t k = 0; k < BLK; k++) p += (double)out[k] * out[k];
    }
    return 10 * log10(p / (n / 2) / 0.5 + 1e-30);
}

static void report_voice_hpf(void)
{
    static const double f[] = { 67.0, 100.0, 150.0, 203.5, 254.1, 300.0, 400.0 };
    printf("info: TX voice HPF gain, dB (voice only; the tone is added after it)\n");
    printf("   Hz      150 Hz edge (today)   300 Hz edge\n");
    for (unsigned i = 0; i < sizeof f / sizeof f[0]; i++)
        printf("  %6.1f   %8.1f              %8.1f\n", f[i],
               hpf_gain_db(150, f[i]), hpf_gain_db(300, f[i]));
}

int main(void)
{
    check_tones();
    check_transitions();
    report_voice_hpf();
    printf(failures ? "%d FAILED\n" : "all ok\n", failures);
    return failures ? 1 : 0;
}
