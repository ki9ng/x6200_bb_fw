/*
 * Host-side check of App/SDR/sdr_compressor.c (TX voice compressor + gate) at
 * its real 48 kHz rate, with board services stubbed (tests/dsp/stubs).
 *
 * Feeds steady 1 kHz tones at stepped levels and measures the settled output
 * level, giving the static curve. Levels are dB of mean power re full scale,
 * the same measure the compressor's detector uses (Sdr_Pow2Db of mean x^2).
 *
 * Expected shape with the defaults (threshold -32 dB, ratio 4, gate ratio
 * 0.25, 24 dB makeup): above the knee, 1 dB in -> 1/ratio dB out; below it,
 * downward expansion at 4 dB/dB until the 40 dB suppression cap; the output
 * never exceeds the unity level (0 dB) even when the input does.
 */
#include <math.h>
#include <stdio.h>
#include "sdr_compressor.h"

#define FS    48000.0
#define BLK   64u
#define NBLK  2250u     /* 3 s: release tau is 250 ms */
#define MEAS  750u      /* measure the last 1 s */

static int failures;

static void expect(const char *what, double got, double lo, double hi)
{
    int ok = got >= lo && got <= hi;
    if (!ok) failures++;
    printf("  %-42s %8.2f  [%s, want %g..%g]\n", what, got,
           ok ? "ok" : "FAIL", lo, hi);
}

/* Settled output level (dB) for a 1 kHz tone at in_db (mean-power dB). */
static double out_level_db(double in_db)
{
    Sdr_Compressor_Init();                 /* fresh state for every point */
    double amp = sqrt(2.0) * pow(10.0, in_db / 20.0);
    float buf[BLK];
    double p = 0.0;
    for (uint32_t b = 0; b < NBLK; b++) {
        for (uint32_t k = 0; k < BLK; k++)
            buf[k] = (float)(amp * sin(2 * M_PI * 1000.0 * (b * BLK + k) / FS));
        Sdr_Compressor_Process(buf, BLK);
        if (b >= NBLK - MEAS)
            for (uint32_t k = 0; k < BLK; k++) p += (double)buf[k] * buf[k];
    }
    return 10 * log10(p / (MEAS * BLK) + 1e-30);
}

int main(void)
{
    printf("static curve, defaults (threshold -32 dB, ratio 4)\n");
    printf("   in dB   out dB\n");
    double in[] = { -80, -60, -50, -44, -40, -36, -34, -30, -26, -20, -14, -8, -3, 0, 6 };
    double out[sizeof in / sizeof in[0]];
    for (unsigned i = 0; i < sizeof in / sizeof in[0]; i++) {
        out[i] = out_level_db(in[i]);
        printf("  %6.1f  %7.2f\n", in[i], out[i]);
    }

    /* indices: -26 -> 10, -8 -> 11 ... use named points */
    double o_m26 = out_level_db(-26), o_m8 = out_level_db(-8);
    expect("slope above knee (-26..-8 dB), dB/dB", (o_m8 - o_m26) / 18.0, 0.20, 0.30);
    double o_m44 = out_level_db(-44), o_m40 = out_level_db(-40);
    expect("slope below knee (-44..-40 dB), dB/dB", (o_m40 - o_m44) / 4.0, 3.5, 4.5);
    expect("output at 0 dB input (unity point), dB", out_level_db(0), -1.0, 0.5);
    expect("output at +6 dB input (limited), dB", out_level_db(6), -1.0, 0.5);
    double o_m80 = out_level_db(-80);
    /* MAX_GATE_SUPPRESSION (40 dB) caps the gate before makeup gain is added,
     * so the net reduction of a quiet signal is 40 - 24 (makeup) = 16 dB.
     * Recorded as measured; whether 16 or 40 net is intended is a question
     * for the author, not asserted as a bug. */
    expect("net suppression at -80 dB input, dB", -80.0 - o_m80, 15.0, 17.0);

    printf("ratio 2\n");
    Sdr_Compressor_Init();
    Sdr_Compressor_SetRatio(2);
    /* SetRatio is lost by out_level_db's re-init, so measure inline here. */
    {
        double r[2], lv[2] = { -26, -8 };
        for (int j = 0; j < 2; j++) {
            double amp = sqrt(2.0) * pow(10.0, lv[j] / 20.0), p = 0;
            float buf[BLK];
            Sdr_Compressor_Init();
            Sdr_Compressor_SetRatio(2);
            for (uint32_t b = 0; b < NBLK; b++) {
                for (uint32_t k = 0; k < BLK; k++)
                    buf[k] = (float)(amp * sin(2 * M_PI * 1000.0 * (b * BLK + k) / FS));
                Sdr_Compressor_Process(buf, BLK);
                if (b >= NBLK - MEAS)
                    for (uint32_t k = 0; k < BLK; k++) p += (double)buf[k] * buf[k];
            }
            r[j] = 10 * log10(p / (MEAS * BLK) + 1e-30);
        }
        expect("slope above knee at ratio 2, dB/dB", (r[1] - r[0]) / 18.0, 0.45, 0.55);
    }

    printf("disabled\n");
    {
        Sdr_Compressor_Init();
        Sdr_Compressor_Enable(false);
        float buf[BLK], ref[BLK];
        double err = 0;
        for (uint32_t k = 0; k < BLK; k++) ref[k] = buf[k] = (float)(0.7 * sin(k * 0.3));
        Sdr_Compressor_Process(buf, BLK);
        for (uint32_t k = 0; k < BLK; k++) err += fabs(buf[k] - ref[k]);
        expect("disabled is bit-exact passthrough (sum err)", err, 0, 0);
    }

    printf(failures ? "FAILED: %d\n" : "all ok\n", failures);
    return failures ? 1 : 0;
}
