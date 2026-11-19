/*
 * Host-side check of App/SDR/sdr_sideband.c, the complex band-pass FIR that
 * does SSB demodulation (RX) and analytic-signal generation (TX) at 12 kHz.
 *
 * RX: a complex tone at +f is upper sideband, at -f lower sideband. USB demod
 *     must pass +f and reject -f; LSB demod the reverse.
 * TX: real audio tone in, complex I/Q out. Energy must sit on the wanted side
 *     (+f for USB, -f for LSB); the opposite side is the unwanted sideband.
 *
 * Gains are steady-state power ratios after the 249-tap FIR has filled.
 *
 * Gain convention (measured, consistent with the coefficient design): the
 * complex taps carry a factor of 2, so one side of a real signal comes out at
 * its original amplitude. A unit complex tone therefore demodulates to a real
 * tone of amplitude 2 (+6 dB against 0.5*|iq|^2 here), and a unit real audio
 * tone modulates to a unit complex tone (+3 dB against the audio power).
 */
#include <math.h>
#include <stdio.h>
#include "arm_math.h"
#include "drv_codec.h"
#include "sdr_sideband.h"

#define FS     12000.0
#define BLK    (DRV_CODEC_BLOCK_FRAMES / 4u)   /* 48 kHz block / 4 = 12 kHz */
#define NBLK   1500u                           /* 2 s */
#define SKIP   100u                            /* > 249 taps of settling */

static int failures;

static void expect(const char *what, double got, double lo, double hi)
{
    int ok = got >= lo && got <= hi;
    if (!ok) failures++;
    printf("  %-40s %9.2f dB  [%s, want %g..%g]\n", what, got,
           ok ? "ok" : "FAIL", lo, hi);
}

/* RX: complex tone exp(j*2*pi*f*t) with signed f, demodulated as USB or LSB. */
static double rx_gain_db(double f, bool lower)
{
    Sdr_Sideband_Reset();
    float i[BLK], q[BLK], audio[BLK];
    Sdr_IQ_Data d = { i, q, BLK };
    double pin = 0, pout = 0;
    for (uint32_t b = 0; b < NBLK; b++) {
        for (uint32_t k = 0; k < BLK; k++) {
            double ph = 2 * M_PI * f * (double)(b * BLK + k) / FS;
            i[k] = (float)cos(ph);
            q[k] = (float)sin(ph);
        }
        Sdr_Sideband_Demod(&d, audio, lower);
        if (b < SKIP) continue;
        for (uint32_t k = 0; k < BLK; k++) {
            pin  += 0.5 * ((double)i[k] * i[k] + (double)q[k] * q[k]);
            pout += (double)audio[k] * audio[k];
        }
    }
    return 10 * log10(pout / pin + 1e-30);
}

/* TX: real tone at f; returns power at +f and -f in the complex output (dB
 * relative to the input tone power) via single-bin correlation. */
static void tx_sides_db(double f, bool lower, double *pos, double *neg)
{
    Sdr_Sideband_Reset();
    float audio[BLK], i[BLK], q[BLK];
    Sdr_IQ_Data d = { i, q, BLK };
    double pr = 0, pi_ = 0, nr = 0, ni = 0, pin = 0;
    uint32_t n = 0;
    for (uint32_t b = 0; b < NBLK; b++) {
        for (uint32_t k = 0; k < BLK; k++)
            audio[k] = (float)sin(2 * M_PI * f * (double)(b * BLK + k) / FS);
        Sdr_Sideband_Mod(audio, &d, lower);
        if (b < SKIP) continue;
        for (uint32_t k = 0; k < BLK; k++, n++) {
            double ph = 2 * M_PI * f * (double)(b * BLK + k) / FS;
            double c = cos(ph), s = sin(ph);
            /* x * exp(-j ph): picks +f.  x * exp(+j ph): picks -f. */
            pr  += i[k] * c + q[k] * s;  pi_ += q[k] * c - i[k] * s;
            nr  += i[k] * c - q[k] * s;  ni  += q[k] * c + i[k] * s;
            pin += (double)audio[k] * audio[k];
        }
    }
    double ppos = (pr * pr + pi_ * pi_) / ((double)n * n);
    double pneg = (nr * nr + ni * ni) / ((double)n * n);
    pin /= n;
    *pos = 10 * log10(ppos / pin + 1e-30);
    *neg = 10 * log10(pneg / pin + 1e-30);
}

int main(void)
{
    static const double tones[] = { 300.0, 1000.0, 2500.0 };
    char label[80];

    printf("RX sideband selection (12 kHz, 249-tap complex FIR)\n");
    for (unsigned t = 0; t < 3; t++) {
        double f = tones[t];
        double usb_want = rx_gain_db(+f, false), usb_rej = rx_gain_db(-f, false);
        double lsb_want = rx_gain_db(-f, true),  lsb_rej = rx_gain_db(+f, true);
        snprintf(label, sizeof label, "USB demod, USB tone %.0f Hz", f);
        expect(label, usb_want, 5.0, 7.0);
        snprintf(label, sizeof label, "USB demod, opposite side %.0f Hz", f);
        expect(label, usb_rej, -300, -60);
        snprintf(label, sizeof label, "LSB demod, LSB tone %.0f Hz", f);
        expect(label, lsb_want, 5.0, 7.0);
        snprintf(label, sizeof label, "LSB demod, opposite side %.0f Hz", f);
        expect(label, lsb_rej, -300, -60);
    }

    printf("TX unwanted-sideband suppression\n");
    for (unsigned t = 0; t < 3; t++) {
        double f = tones[t], pos, neg;
        tx_sides_db(f, false, &pos, &neg);
        snprintf(label, sizeof label, "USB mod %.0f Hz: wanted (+f)", f);
        expect(label, pos, 2.0, 4.0);
        snprintf(label, sizeof label, "USB mod %.0f Hz: suppression", f);
        expect(label, neg - pos, -300, -60);
        tx_sides_db(f, true, &pos, &neg);
        snprintf(label, sizeof label, "LSB mod %.0f Hz: suppression", f);
        expect(label, pos - neg, -300, -60);
    }

    printf(failures ? "FAILED: %d\n" : "all ok\n", failures);
    return failures ? 1 : 0;
}
