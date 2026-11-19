#include "sdr_tx.h"

#include <stdbool.h>
#include <stdint.h>

#include <arm_math.h>

#include "board_config.h"
#include "bsp_log.h"
#include "bsp_irq.h"
#include "app_event.h"
#include "drv_codec.h"
#include "sdr_audio_resample.h"
#include "sdr_iir.h"
#include "sdr_resample.h"
#include "sdr_am.h"
#include "sdr_ssb.h"
#include "sdr_nbfm.h"
#include "sdr_cw.h"
#include "sdr_compressor.h"
#include "sdr_soft_clipper.h"

#define LOG_TAG "app.sdr.tx"

#define SDR_TX_FILTER_NUM_STAGES SDR_IIR_STAGES_5

typedef struct {
    bool     needs_audio;
    uint32_t dsp_rate_hz;   /* modulator sample rate; 48 kHz means no resampling */
} SdrTxPlan;

static const SdrTxPlan k_tx_plans[SDR_MOD_NFM + 1u] = {
    [SDR_MOD_LSB]   = { true,  SDR_TX_RATE_12K },
    [SDR_MOD_LSB_D] = { true,  SDR_TX_RATE_12K },
    [SDR_MOD_USB]   = { true,  SDR_TX_RATE_12K },
    [SDR_MOD_USB_D] = { true,  SDR_TX_RATE_12K },
    /* CW/CWR: no audio, generated directly at 48 kHz by Sdr_CW_Mod. */
    [SDR_MOD_CW]    = { false, SDR_TX_RATE_48K },
    [SDR_MOD_CWR]   = { false, SDR_TX_RATE_48K },
    /* AM/SAM: audio at 48 kHz, real output (Q = 0). */
    [SDR_MOD_AM]    = { true,  SDR_TX_RATE_48K },
    [SDR_MOD_SAM]   = { true,  SDR_TX_RATE_48K },
    /* NBFM: audio at 48 kHz, complex output. */
    [SDR_MOD_NFM]   = { true,  SDR_TX_RATE_48K },
};

static arm_biquad_casd_df1_inst_f32 s_lpf_inst;
static float s_lpf_coeffs[SDR_TX_FILTER_NUM_STAGES * 5];
static float s_lpf_state[SDR_TX_FILTER_NUM_STAGES * 4];
static arm_biquad_casd_df1_inst_f32 s_hpf_inst;
static float s_hpf_coeffs[SDR_TX_FILTER_NUM_STAGES * 5];
static float s_hpf_state[SDR_TX_FILTER_NUM_STAGES * 4];
static uint32_t s_filter_low_hz  = 150u;
static uint32_t s_filter_high_hz = 2850u;

static SdrSoftClipperConfig s_clipper_voice_cfg;
static SdrSoftClipperConfig s_clipper_am_cfg;

static float s_audio_bpf[DRV_CODEC_BLOCK_FRAMES] __attribute__((aligned(32)));
static float s_audio_dsp[SDR_AUDIO_BLOCK_24K] __attribute__((aligned(32)));
static float s_iq_i[DRV_CODEC_BLOCK_FRAMES] __attribute__((aligned(32)));
static float s_iq_q[DRV_CODEC_BLOCK_FRAMES] __attribute__((aligned(32)));

static struct {
    AppRadioMode radio_mode;
} s_state;

static struct {
    bool reset_bpf;
} s_pending;

static void tx_design_bpf(uint32_t low_hz, uint32_t high_hz)
{
    SdrIirSpec spec = {
        .edge = SDR_IIR_EDGE_NOISE_BW,
        .f_edge_hz = (float)low_hz,
        .fs_hz = (float)SDR_TX_RATE_48K,
        .stages = SDR_TX_FILTER_NUM_STAGES,
        .type  = SDR_IIR_TYPE_HPF,
    };
    uint8_t stages = 0;

    if (Sdr_Iir_DesignEllip(&spec, s_hpf_coeffs, &stages) != SDR_IIR_OK) {
        LOG_WARN("TX HPF design failed (%u Hz)", (unsigned)low_hz);
        return;
    }
    arm_biquad_cascade_df1_init_f32(&s_hpf_inst, SDR_TX_FILTER_NUM_STAGES,
                                    s_hpf_coeffs, s_hpf_state);

    spec.f_edge_hz = (float)high_hz;
    spec.type  = SDR_IIR_TYPE_LPF;
    if (Sdr_Iir_DesignEllip(&spec, s_lpf_coeffs, &stages) != SDR_IIR_OK) {
        LOG_WARN("TX LPF design failed (%u Hz)", (unsigned)high_hz);
        return;
    }
    arm_biquad_cascade_df1_init_f32(&s_lpf_inst, SDR_TX_FILTER_NUM_STAGES,
                                    s_lpf_coeffs, s_lpf_state);
}

static void reset_bpf(void) {
    arm_fill_f32(0.0f, s_hpf_state, SDR_TX_FILTER_NUM_STAGES * 4);
    arm_fill_f32(0.0f, s_lpf_state, SDR_TX_FILTER_NUM_STAGES * 4);
}

static void on_rx_tx(const AppEventRxTxEvent *evt, void *user) {
    (void)user;
    if (evt->mode == APP_RADIO_MODE_RX) {
        // Leaving TX
        uint32_t key = Bsp_Irq_SaveLow();
        s_pending.reset_bpf = true;
        Bsp_Irq_RestoreLow(key);
    }
    s_state.radio_mode = evt->mode;
    Sdr_AudioResample_Reset();
}

void Sdr_Tx_Init(void)
{
    tx_design_bpf(s_filter_low_hz, s_filter_high_hz);
    Sdr_Compressor_Init();

    // Soft clip voice
    Sdr_SoftClipper_Setup(&s_clipper_voice_cfg, 2.0f, 3.0f, false);
    // Asymmetric clipper for AM
    Sdr_SoftClipper_Setup(&s_clipper_am_cfg, 0.95f, 1.0f, true);

    if (App_Event_SubscribeRxTx(on_rx_tx, NULL)) {
        LOG_ERROR("App_Event_SubscribeRxTx failed");
    }
}

void Sdr_Tx_SetFilterFreq(uint32_t low_hz, uint32_t high_hz)
{
    if (low_hz == s_filter_low_hz && high_hz == s_filter_high_hz) {
        return;
    }
    s_filter_low_hz  = low_hz;
    s_filter_high_hz = high_hz;
    tx_design_bpf(low_hz, high_hz);
}

bool Sdr_Tx_ModeSupported(SdrModulation mode)
{
    switch (mode) {
    case SDR_MOD_LSB:
    case SDR_MOD_LSB_D:
    case SDR_MOD_USB:
    case SDR_MOD_USB_D:
    case SDR_MOD_CW:
    case SDR_MOD_CWR:
    case SDR_MOD_AM:
    case SDR_MOD_SAM:
    case SDR_MOD_NFM:
        return true;
    default:
        return false;
    }
}

static void apply_pending(void) {
    if (s_pending.reset_bpf) {
        s_pending.reset_bpf = false;
        reset_bpf();
    }
}

void Sdr_Tx_Process(const float *audio48, uint32_t n, SdrModulation mode,
                    float *out_i, float *out_q, float *monitor)
{
    if (out_i == 0 || out_q == 0) {
        return;
    }

    apply_pending();

    const SdrTxPlan *plan = (mode <= SDR_MOD_NFM) ? &k_tx_plans[mode] : 0;
    bool supported = plan && ((audio48 && plan->needs_audio) || !plan->needs_audio) && (n == DRV_CODEC_BLOCK_FRAMES);

    if (!supported) {
        for (uint32_t i = 0; i < n; i++) {
            out_i[i] = 0.0f;
            out_q[i] = 0.0f;
        }
        return;
    }

    float *audio_dsp = s_audio_bpf;
    uint32_t audio_n = n;

    if (plan->needs_audio) {
        if (mode == SDR_MOD_NFM && Sdr_NBFM_IsFlat()) {
            /* Flat FM (data, USB audio): no voice HPF, compressor or clipper.
             * The LPF below still bounds the occupied bandwidth. */
            arm_copy_f32(audio48, s_audio_bpf, n);
        } else {
            arm_biquad_cascade_df1_f32(&s_hpf_inst, audio48, s_audio_bpf, n);
            /* Perhaps, disable compressor and clipper for digital modes */
            Sdr_Compressor_Process(s_audio_bpf, n);
            Sdr_SoftClipper_Process(&s_clipper_voice_cfg, s_audio_bpf, s_audio_bpf, n);
        }
        arm_biquad_cascade_df1_f32(&s_lpf_inst, s_audio_bpf, s_audio_bpf, n);
        arm_copy_f32(s_audio_bpf, monitor, n);

        /* Resample the audio down to the modulator rate when needed. */
        if (plan->dsp_rate_hz < SDR_TX_RATE_48K) {
            // Audio already filtered, only decimation
            const uint8_t step = SDR_TX_RATE_48K / plan->dsp_rate_hz;
            for (size_t i = 0; i < n / step; i++) {
                s_audio_dsp[i] = s_audio_bpf[step * i];
            }
            audio_n = n / step;
            audio_dsp = s_audio_dsp;
        }
    } else {
        arm_fill_f32(0.0f, monitor, n);
    }

    Sdr_IQ_Data iq;
    if (plan->dsp_rate_hz < SDR_TX_RATE_48K) {
        iq.i    = s_iq_i;
        iq.q    = s_iq_q;
    } else {
        iq.i    = out_i;
        iq.q    = out_q;

    }
    iq.size = audio_n;

    switch (mode) {
    case SDR_MOD_LSB:
    case SDR_MOD_LSB_D:
        Sdr_LSB_Mod(audio_dsp, &iq);
        break;
    case SDR_MOD_USB:
    case SDR_MOD_USB_D:
        Sdr_USB_Mod(audio_dsp, &iq);
        break;
    case SDR_MOD_CW:
    case SDR_MOD_CWR:
        Sdr_CW_Mod(NULL, &iq);
        break;
    case SDR_MOD_AM:
    case SDR_MOD_SAM:
        /* Amplitude after compressor and voice clipper can be up to +3 dB above 0 */
        arm_scale_f32(audio_dsp, 0.707f, audio_dsp, audio_n);
        Sdr_SoftClipper_Process(&s_clipper_am_cfg, audio_dsp, audio_dsp, n);
        Sdr_AM_Mod(audio_dsp, &iq);
        break;
        case SDR_MOD_NFM:
        /* Amplitude after compressor and voice clipper can be up to +3 dB above 0 */
        arm_scale_f32(audio_dsp, 0.707f, audio_dsp, audio_n);
        Sdr_NBFM_Mod(audio_dsp, &iq);
        break;
    default:
        for (uint32_t i = 0; i < n; i++) {
            out_i[i] = 0.0f;
            out_q[i] = 0.0f;
        }
        return;
    }

    /* Resample the IQ result back to 48 kHz when needed. */
    if (plan->dsp_rate_hz < SDR_TX_RATE_48K) {
        Sdr_IQ_Data up;
        up.i = out_i;
        up.q = out_q;
        Sdr_Interpolate_Process(iq, &up, plan->dsp_rate_hz);
        if (up.size != n) {
            for (uint32_t i = 0; i < n; i++) {
                out_i[i] = 0.0f;
                out_q[i] = 0.0f;
            }
            return;
        }
    }
}
