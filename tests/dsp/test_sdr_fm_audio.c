/*
 * Host check of the NBFM audio path switch (voice / flat) together with CTCSS.
 *
 * TX runs the real Sdr_Tx_Process (sdr_tx.c: voice HPF, compressor, clipper,
 * LPF) into the real Sdr_NBFM_Mod; the test discriminates the 48 kHz I/Q to
 * deviation in Hz. RX feeds synthetic FM I/Q of known deviation into the real
 * Sdr_NBFM_Demod and measures the audio. The SSB/AM/CW modulators and the
 * resamplers are not used by NFM and are stubbed below.
 *
 * Voice path: sdr_tx's 150 Hz HPF and compressor act; with CTCSS on, the
 * 300 Hz tone HPF keeps voice out of the tone band; RX removes sub-audible
 * tones unless the RX tone filter is switched off.
 * Flat path: no HPF, compressor, clipper or emphasis on TX, none on RX;
 * CTCSS still works on top.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "sdr_tx.h"
#include "sdr_nbfm.h"
#include "sdr_ctcss.h"
#include "app_event.h"

#define FS    48000u
#define BLK   64u
#define NBLK  1125u             /* 1.5 s */
#define N     (BLK * NBLK)
#define SKIP  (FS / 2)          /* measure the last 1 s: compressor settles */
#define M     (N - SKIP)

/* Not used by NFM. */
void Sdr_LSB_Mod(const float *a, Sdr_IQ_Data *d) { (void)a; (void)d; }
void Sdr_USB_Mod(const float *a, Sdr_IQ_Data *d) { (void)a; (void)d; }
void Sdr_CW_Mod(const float *a, Sdr_IQ_Data *d) { (void)a; (void)d; }
void Sdr_AM_Mod(const float *a, Sdr_IQ_Data *d) { (void)a; (void)d; }
void Sdr_Interpolate_Process(Sdr_IQ_Data s, Sdr_IQ_Data *d, uint32_t f) { (void)s; (void)f; d->size = 0; }
void Sdr_AudioResample_Reset(void) {}

extern AppEventRxCfgFn stub_rx_cfg_cb;
extern void *stub_rx_cfg_user;

static int failures;

static void expect(const char *what, double got, double lo, double hi)
{
    int ok = got >= lo && got <= hi;
    if (!ok) failures++;
    printf("  %-48s %9.2f  [%s, want %g..%g]\n", what, got,
           ok ? "ok" : "FAIL", lo, hi);
}

static float s_i[N], s_q[N], s_x[N], s_mon[BLK];

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

static double db(double a, double b) { return 20 * log10(a / b + 1e-30); }

static void setup(SdrNbfmAudio path, uint16_t tone_dhz)
{
    Sdr_Tx_Init();
    Sdr_NBFM_Init();
    SdrRxCfg cfg = { .rx_mode = SDR_MOD_NFM, .filter_low_hz = -7500,
                     .filter_high_hz = 7500, .fftdec = 0 };
    stub_rx_cfg_cb(&cfg, stub_rx_cfg_user);         /* 15 kHz band, +-5 kHz */
    Sdr_NBFM_SetAudio(path);
    if (tone_dhz) Sdr_Ctcss_Set(tone_dhz, 0);
}

/* TX: audio = sum of up to two sines (amp a1 at f1, a2 at f2) through
 * Sdr_Tx_Process; s_x gets the deviation in Hz. */
static void tx(double a1, double f1, double a2, double f2)
{
    float a[BLK];
    for (uint32_t b = 0; b < NBLK; b++) {
        for (uint32_t k = 0; k < BLK; k++) {
            double t = (double)(b * BLK + k) / FS;
            a[k] = (float)(a1 * sin(2 * M_PI * f1 * t) + a2 * sin(2 * M_PI * f2 * t));
        }
        Sdr_Tx_Process(a, BLK, SDR_MOD_NFM, s_i + b * BLK, s_q + b * BLK, s_mon);
    }
    s_x[0] = 0;
    for (uint32_t n = 1; n < N; n++) {
        double re = s_i[n] * s_i[n - 1] + s_q[n] * s_q[n - 1];
        double im = s_q[n] * s_i[n - 1] - s_i[n] * s_q[n - 1];
        s_x[n] = (float)(atan2(im, re) * FS / (2 * M_PI));
    }
}

static double tx_dev(double a, double f)
{
    tx(a, f, 0, 0);
    return tone_amp(s_x + SKIP, M, f);
}

/* RX: synthetic FM, deviation d1 Hz at f1 plus d2 Hz at f2, through
 * Sdr_NBFM_Demod; s_x gets the audio. */
static void rx(double d1, double f1, double d2, double f2)
{
    double ph = 0;
    float ci[BLK], cq[BLK];
    for (uint32_t b = 0; b < NBLK; b++) {
        for (uint32_t k = 0; k < BLK; k++) {
            double t = (double)(b * BLK + k) / FS;
            double dev = d1 * sin(2 * M_PI * f1 * t) + d2 * sin(2 * M_PI * f2 * t);
            ph += 2 * M_PI * dev / FS;
            ci[k] = (float)(0.5 * cos(ph));
            cq[k] = (float)(0.5 * sin(ph));
        }
        Sdr_IQ_Data iq = { .i = ci, .q = cq, .size = BLK };
        Sdr_NBFM_Demod(&iq, s_x + b * BLK);
    }
}

static double rx_amp(double d, double f)
{
    rx(d, f, 0, 0);
    return tone_amp(s_x + SKIP, M, f);
}

int main(void)
{
    double v1k, v100, f1k, f100, f150;

    printf("TX voice path, no CTCSS (sdr_tx HPF 150 Hz, compressor on)\n");
    setup(SDR_NBFM_AUDIO_VOICE, 0);
    v1k = tx_dev(0.3, 1000.0);
    v100 = tx_dev(0.3, 100.0);
    printf("  1 kHz deviation %.0f Hz\n", v1k);
    expect("100 Hz vs 1 kHz, dB (HPF cuts it)", db(v100, v1k), -200, -60);
    /* 0.3 and 0.095 are -13.5 / -23.5 dB mean power: both above the -32 dB
     * threshold, so ratio 4 gives about 2.5 dB out for 10 dB in. */
    double hi = tx_dev(0.3, 1000.0), lo = tx_dev(0.3 / sqrt(10.0), 1000.0);
    expect("10 dB input step -> output step, dB (compressed)", db(hi, lo), 0, 5);

    printf("TX flat path, no CTCSS\n");
    setup(SDR_NBFM_AUDIO_FLAT, 0);
    f1k = tx_dev(0.3, 1000.0);
    f100 = tx_dev(0.3, 100.0);
    printf("  1 kHz deviation %.0f Hz\n", f1k);
    expect("100 Hz vs 1 kHz, dB (flat)", db(f100, f1k), -0.1, 0.1);
    hi = f1k; lo = tx_dev(0.3 / sqrt(10.0), 1000.0);
    expect("10 dB input step -> output step, dB (linear)", db(hi, lo), 9.9, 10.1);
    expect("1 kHz deviation, Hz (0.3 x 0.707 x 5 kHz)", f1k, 1050, 1071);

    printf("TX flat path + 100.0 Hz CTCSS, 150 Hz audio\n");
    setup(SDR_NBFM_AUDIO_FLAT, 1000);
    tx(0.3, 150.0, 0, 0);
    expect("CTCSS deviation, Hz", tone_amp(s_x + SKIP, M, 100.0), 594, 606);
    f150 = tone_amp(s_x + SKIP, M, 150.0);
    expect("150 Hz audio vs flat 1 kHz, dB", db(f150, f1k), -0.1, 0.1);

    printf("TX voice path + 100.0 Hz CTCSS (tone HPF 300 Hz)\n");
    setup(SDR_NBFM_AUDIO_VOICE, 0);
    double v203_off = (tx(0.3, 203.5, 0, 0), tone_amp(s_x + SKIP, M, 203.5));
    setup(SDR_NBFM_AUDIO_VOICE, 1000);
    tx(0.3, 203.5, 0, 0);
    double v203_on = tone_amp(s_x + SKIP, M, 203.5);
    expect("CTCSS deviation, Hz", tone_amp(s_x + SKIP, M, 100.0), 594, 606);
    printf("  203.5 Hz voice deviation without CTCSS %.0f Hz\n", v203_off);
    expect("203.5 Hz voice, CTCSS on vs off, dB", db(v203_on, v203_off), -200, -60);
    double v1k_on = tx_dev(0.3, 1000.0);
    expect("1 kHz voice, CTCSS on vs off, dB", db(v1k_on, v1k), -0.5, 0.5);

    printf("RX voice path (100.0 Hz at 600 Hz dev vs 1 kHz at 3 kHz dev)\n");
    setup(SDR_NBFM_AUDIO_VOICE, 0);
    double r1k = rx_amp(3000, 1000.0);
    expect("tone in audio, tone filter on, dB", db(rx_amp(600, 100.0), r1k), -200, -60);
    Sdr_NBFM_SetRxToneFilter(false);
    double r1k_off = rx_amp(3000, 1000.0);
    double r100_off = rx_amp(600, 100.0);
    expect("tone in audio, tone filter off, dB", db(r100_off, r1k_off), -10, 0);
    Sdr_NBFM_SetRxToneFilter(true);
    expect("1 kHz voice, tone filter on vs off, dB", db(r1k, r1k_off), -0.5, 0.5);

    /* Limits from the two filters that stay in the flat path: the 40 Hz
     * first-order DC block (-1.3 dB at 67 Hz, -0.6 dB at 100 Hz, about
     * -0.1 dB from 254 Hz up) and the 3 kHz audio LPF's passband ripple
     * (a few tenths of a dB against the 1 kHz reference). */
    printf("RX flat path: audio level vs 1 kHz at equal deviation (1 kHz dev)\n");
    setup(SDR_NBFM_AUDIO_FLAT, 0);
    double ref = rx_amp(1000, 1000.0);
    static const double fr[] = { 67.0, 100.0, 254.1, 300.0, 2000.0 };
    for (unsigned i = 0; i < sizeof fr / sizeof fr[0]; i++) {
        char what[64];
        snprintf(what, sizeof what, "%.1f Hz, dB", fr[i]);
        expect(what, db(rx_amp(1000, fr[i]), ref), -2.0, 0.5);
    }

    printf(failures ? "%d FAILED\n" : "all ok\n", failures);
    return failures ? 1 : 0;
}
