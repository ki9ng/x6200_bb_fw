/*
 * End-to-end host check of CTCSS through App/SDR/sdr_nbfm.c: the real NBFM
 * modulator (with the CTCSS hook) produces 48 kHz I/Q, and the test
 * discriminates it itself (angle of z[n] * conj(z[n-1])) to get the
 * instantaneous deviation in Hz.
 *
 * Checks: tone deviation on the 15 kHz (+-5 kHz) and 10 kHz (+-2.5 kHz)
 * bands, tone frequency, 1 kHz voice unchanged by the tone (within the
 * passband ripple of the 300 Hz voice HPF that comes on with CTCSS) and the
 * tone by voice, and a clean end when the tone is turned off mid-TX.
 *
 * Information (not pass/fail): the same I/Q looped back through
 * Sdr_NBFM_Demod (voice path, RX tone filter on by default), giving the
 * received tone level relative to a 1 kHz voice tone at 60 % deviation.
 * test_sdr_fm_audio checks the RX tone filter on and off.
 *
 * Board services are stubbed (tests/dsp/stubs); the CORDIC stub uses libm.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "sdr_nbfm.h"
#include "sdr_ctcss.h"
#include "app_event.h"

#define FS    48000u
#define BLK   64u
#define NBLK  750u              /* 1 s */
#define N     (BLK * NBLK)

extern AppEventRxCfgFn stub_rx_cfg_cb;
extern void *stub_rx_cfg_user;

static int failures;

static void expect(const char *what, double got, double lo, double hi)
{
    int ok = got >= lo && got <= hi;
    if (!ok) failures++;
    printf("  %-44s %10.3f  [%s, want %g..%g]\n", what, got,
           ok ? "ok" : "FAIL", lo, hi);
}

static float s_i[N], s_q[N], s_dev[N], s_audio[N];

static void set_band(int32_t width_hz)
{
    SdrRxCfg cfg = { .rx_mode = SDR_MOD_NFM, .filter_low_hz = -width_hz / 2,
                     .filter_high_hz = width_hz / 2, .fftdec = 0 };
    stub_rx_cfg_cb(&cfg, stub_rx_cfg_user);
}

/* Modulate 1 s: audio is a sine of amplitude va at vf Hz (va = 0: silence).
 * stop_at_block >= 0 turns the tone off before that block. */
static void modulate(double va, double vf, int stop_at_block)
{
    float a[BLK];
    for (uint32_t b = 0; b < NBLK; b++) {
        if ((int)b == stop_at_block) Sdr_Ctcss_Set(0, 0);
        for (uint32_t k = 0; k < BLK; k++)
            a[k] = (float)(va * sin(2 * M_PI * vf * (b * BLK + k) / FS));
        Sdr_IQ_Data iq = { .i = s_i + b * BLK, .q = s_q + b * BLK, .size = BLK };
        Sdr_NBFM_Mod(a, &iq);
    }
    s_dev[0] = 0.0f;
    for (uint32_t n = 1; n < N; n++) {
        double re = s_i[n] * s_i[n - 1] + s_q[n] * s_q[n - 1];
        double im = s_q[n] * s_i[n - 1] - s_i[n] * s_q[n - 1];
        s_dev[n] = (float)(atan2(im, re) * FS / (2 * M_PI));
    }
}

/* Hann-windowed single-bin amplitude of a sine at f over x[a..a+n). */
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

static double peak_abs(const float *x, uint32_t n)
{
    double m = 0;
    for (uint32_t i = 0; i < n; i++) if (fabs(x[i]) > m) m = fabs(x[i]);
    return m;
}

/* Fresh modulator + band + tone. The first Mod call runs the pending TX
 * reset, which restarts the tone at zero phase. */
static void start(int32_t band_hz, uint16_t tone_dhz)
{
    Sdr_NBFM_Init();
    set_band(band_hz);
    if (tone_dhz) Sdr_Ctcss_Set(tone_dhz, 0);
}

static void demod(void)
{
    static float ci[BLK], cq[BLK];
    for (uint32_t b = 0; b < NBLK; b++) {
        memcpy(ci, s_i + b * BLK, sizeof ci);
        memcpy(cq, s_q + b * BLK, sizeof cq);
        Sdr_IQ_Data iq = { .i = ci, .q = cq, .size = BLK };
        Sdr_NBFM_Demod(&iq, s_audio + b * BLK);
    }
}

int main(void)
{
    const uint32_t skip = FS / 10;          /* ignore the first 100 ms */
    const uint32_t m = N - skip;

    printf("15 kHz band (+-5 kHz), 100.0 Hz tone, no voice\n");
    start(15000, 1000);
    modulate(0.0, 0.0, -1);
    expect("tone deviation, Hz", tone_amp(s_dev + skip, m, 100.0), 594, 606);
    expect("peak deviation, Hz", peak_abs(s_dev + skip, m), 594, 606);
    expect("tone frequency, Hz", zc_freq(s_dev + skip, m), 99.99, 100.01);

    printf("10 kHz band (+-2.5 kHz), 100.0 Hz tone, no voice\n");
    start(10000, 1000);
    modulate(0.0, 0.0, -1);
    expect("tone deviation, Hz", tone_amp(s_dev + skip, m, 100.0), 297, 303);

    printf("15 kHz band, 1 kHz voice at 0.5, with and without 100.0 Hz tone\n");
    start(15000, 0);
    modulate(0.5, 1000.0, -1);
    double voice_alone = tone_amp(s_dev + skip, m, 1000.0);
    start(15000, 1000);
    modulate(0.5, 1000.0, -1);
    double voice_with = tone_amp(s_dev + skip, m, 1000.0);
    printf("  voice deviation alone %.1f Hz\n", voice_alone);
    expect("voice change with tone on, dB", 20 * log10(voice_with / voice_alone), -0.1, 0.1);
    expect("tone deviation under voice, Hz", tone_amp(s_dev + skip, m, 100.0), 594, 606);

    printf("tone turned off at 0.5 s, no voice\n");
    start(15000, 1000);
    modulate(0.0, 0.0, NBLK / 2);
    expect("peak deviation in last 0.4 s, Hz", peak_abs(s_dev + N - FS * 4 / 10, FS * 4 / 10), 0, 0.5);

    printf("info: RX loopback through Sdr_NBFM_Demod, 15 kHz band\n");
    start(15000, 0);
    modulate(0.6, 1000.0, -1);              /* 0.6 of +-5 kHz = 3 kHz deviation */
    demod();
    double v_rx = tone_amp(s_audio + skip, m, 1000.0);
    start(15000, 1000);
    modulate(0.0, 0.0, -1);
    demod();
    double t_rx = tone_amp(s_audio + skip, m, 100.0);
    printf("  100.0 Hz tone (600 Hz dev) vs 1 kHz voice (3 kHz dev) in RX audio: %.1f dB\n",
           20 * log10(t_rx / v_rx + 1e-30));

    printf(failures ? "%d FAILED\n" : "all ok\n", failures);
    return failures ? 1 : 0;
}
