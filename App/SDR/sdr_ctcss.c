#include "sdr_ctcss.h"

#include <arm_math.h>

#include "bsp_irq.h"
#include "bsp_log.h"

#define LOG_TAG "app.sdr.ctcss"

/* One phase-accumulator turn is 2^32. */
#define SDR_CTCSS_RAD_PER_LSB ((float)(2.0 * M_PI / 4294967296.0))

typedef struct {
    uint32_t inc;   /* phase step per sample; 0 = off */
    float    amp;   /* peak, 1.0 = channel peak deviation */
} SdrCtcssTone;

static uint32_t     s_fs_hz;

/* Modulator (ISR) state. */
static SdrCtcssTone s_tone;
static uint32_t     s_phase;
static float        s_prev;
static bool         s_stopping;

static struct {
    bool         update;
    SdrCtcssTone tone;
} s_pending;

static void stop_now(void) {
    s_tone.inc = 0u;
    s_tone.amp = 0.0f;
    s_stopping = false;
    s_phase = 0u;
    s_prev = 0.0f;
}

static void apply_pending(void) {
    if (!s_pending.update) {
        return;
    }
    s_pending.update = false;

    SdrCtcssTone t = s_pending.tone;
    if (t.inc == 0u) {
        if (s_tone.inc != 0u) {
            s_stopping = true;
        }
        return;
    }
    if (s_tone.inc == 0u) {
        s_phase = 0u;
        s_prev = 0.0f;
    }
    s_stopping = false;
    s_tone = t;
}

void Sdr_Ctcss_Init(uint32_t fs_hz) {
    s_fs_hz = fs_hz;
    s_pending.update = false;
    stop_now();
}

int Sdr_Ctcss_Set(uint16_t freq_dhz, uint16_t dev_permille) {
    SdrCtcssTone t = { 0u, 0.0f };

    if (dev_permille == 0u) {
        dev_permille = SDR_CTCSS_DEV_DEFAULT_PERMILLE;
    }
    if (dev_permille > SDR_CTCSS_DEV_MAX_PERMILLE) {
        return -1;
    }
    if (freq_dhz != 0u) {
        if (freq_dhz < SDR_CTCSS_FREQ_MIN_DHZ || freq_dhz > SDR_CTCSS_FREQ_MAX_DHZ ||
            s_fs_hz == 0u) {
            return -1;
        }
        /* inc = f / fs * 2^32, rounded; f = freq_dhz / 10. */
        uint64_t den = (uint64_t)s_fs_hz * 10u;
        t.inc = (uint32_t)((((uint64_t)freq_dhz << 32) + den / 2u) / den);
        t.amp = (float)dev_permille / 1000.0f;
    }

    uint32_t key = Bsp_Irq_SaveLow();
    s_pending.tone = t;
    s_pending.update = true;
    Bsp_Irq_RestoreLow(key);

    LOG_DEBUG("ctcss %u.%u Hz, dev %u/1000", (unsigned)(freq_dhz / 10u),
              (unsigned)(freq_dhz % 10u), (unsigned)dev_permille);
    return 0;
}

void Sdr_Ctcss_Reset(void) {
    apply_pending();
    if (s_stopping) {
        stop_now();
        return;
    }
    s_phase = 0u;
    s_prev = 0.0f;
}

bool Sdr_Ctcss_Fill(float *out, uint32_t n) {
    apply_pending();
    if (s_tone.inc == 0u) {
        return false;
    }

    for (uint32_t i = 0; i < n; i++) {
        float s = s_tone.amp * arm_sin_f32((float)s_phase * SDR_CTCSS_RAD_PER_LSB);

        if (s_stopping && ((s_prev > 0.0f && s <= 0.0f) || (s_prev < 0.0f && s >= 0.0f))) {
            for (; i < n; i++) {
                out[i] = 0.0f;
            }
            stop_now();
            return true;
        }
        out[i] = s;
        s_prev = s;
        s_phase += s_tone.inc;
    }
    return true;
}
