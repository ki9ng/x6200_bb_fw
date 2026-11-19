/*
 * Host-side check of App/SDR/sdr_agc.c (RX AGC) at its real 24 kHz rate and
 * 32-sample block, with board services, the spectrum noise estimate and the AM
 * carrier stubbed (tests/dsp/stubs).
 *
 * The AGC's static law, read from the source: with threshold = noise + 16 dB,
 *   below threshold : gain = target - threshold      (fixed, noise held level)
 *   above threshold : out  = target + 0.1 * (env - threshold)
 * clamped to -20..110 dB of gain; with AGC off the gain is a fixed 70 dB.
 * Levels are peak dB (Sdr_Lin2Db of the block peak), as the AGC measures them.
 */
#include <math.h>
#include <stdio.h>
#include "drv_codec.h"
#include "sdr_agc.h"
#include "sdr_config.h"
#include "app_event.h"

#define FS      24000.0
#define BLK     (DRV_CODEC_BLOCK_FRAMES / 2u)   /* 32 at 24 kHz */
#define SETTLE  (15u * 750u)                    /* 15 s: noise tracker tau 1 s */
#define MEAS    750u                            /* last 1 s */

extern float stub_noise_db;
extern AppEventRxCfgFn stub_rx_cfg_cb;
extern void *stub_rx_cfg_user;

static int failures;

static void expect(const char *what, double got, double lo, double hi)
{
    int ok = got >= lo && got <= hi;
    if (!ok) failures++;
    printf("  %-44s %8.2f  [%s, want %g..%g]\n", what, got,
           ok ? "ok" : "FAIL", lo, hi);
}

static void setup(SdrAgcMode mode, float noise_db)
{
    stub_noise_db = noise_db;
    Sdr_AGC_Init();
    SdrRxCfg cfg = { .rx_mode = SDR_MOD_USB };
    stub_rx_cfg_cb(&cfg, stub_rx_cfg_user);
    Sdr_AGC_SetMode(mode);
}

/* Run a 1 kHz tone of peak level in_db; return settled output peak in dB. */
static double out_peak_db(SdrAgcMode mode, float noise_db, double in_db)
{
    setup(mode, noise_db);
    double amp = pow(10.0, in_db / 20.0), peak = 0.0;
    float buf[BLK];
    for (uint32_t b = 0; b < SETTLE + MEAS; b++) {
        for (uint32_t k = 0; k < BLK; k++)
            buf[k] = (float)(amp * sin(2 * M_PI * 1000.0 * (b * BLK + k) / FS));
        Sdr_AGC_Process(buf, BLK);
        if (b >= SETTLE)
            for (uint32_t k = 0; k < BLK; k++)
                if (fabs(buf[k]) > peak) peak = fabs(buf[k]);
    }
    return 20 * log10(peak + 1e-30);
}

int main(void)
{
    const float noise = -100.0f;           /* threshold = -84 dB */
    printf("SSB, AGC slow, noise %.0f dB (threshold %.0f dB)\n", noise, noise + 16);
    printf("   in dB   out dB   expected\n");
    double in[] = { -95, -88, -80, -60, -40, -20, -5 };
    for (unsigned i = 0; i < sizeof in / sizeof in[0]; i++) {
        double o = out_peak_db(SDR_AGC_SLOW, noise, in[i]);
        double thr = noise + 16.0;
        double want = in[i] <= thr ? in[i] - thr : 0.1 * (in[i] - thr);
        if (in[i] - thr > 0 && want - in[i] < -20.0) want = in[i] - 20.0;
        printf("  %6.1f  %7.2f  %7.2f\n", in[i], o, want);
        char label[64];
        snprintf(label, sizeof label, "in %.0f dB: out - expected", in[i]);
        expect(label, o - want, -1.0, 1.0);
    }

    printf("gain limits\n");
    /* -20 dB gain floor: a +40 dB peak cannot be pulled below +20 dB. */
    expect("in +40 dB (gain floor -20): out dB",
           out_peak_db(SDR_AGC_SLOW, noise, 40.0), 19.0, 21.0);

    printf("AGC off\n");
    expect("in -80 dB, AGC off (fixed 70 dB): out dB",
           out_peak_db(SDR_AGC_OFF, noise, -80.0), -10.5, -9.5);

    printf(failures ? "FAILED: %d\n" : "all ok\n", failures);
    return failures ? 1 : 0;
}
