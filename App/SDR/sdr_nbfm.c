#include "sdr_nbfm.h"

#include <arm_math.h>
#include "board_config.h"
#include "bsp_log.h"
#include "bsp_irq.h"
#include "bsp_cordic.h"
#include "drv_codec.h"
#include "sdr_ctcss.h"
#include "sdr_fm_disc.h"
#include "sdr_nbfm_filters.h"
#include "app_event.h"
#include "sdr_iir.h"
#include "sdr_spectrum.h"
#include "sdr_squelch_filters.h"

#define LOG_TAG "app.sdr.nbfm"

#define SDR_NBFM_SAMPLE_RATE_DIVISOR 1u
#define SDR_NBFM_BLOCK_SIZE (DRV_CODEC_BLOCK_FRAMES / SDR_NBFM_SAMPLE_RATE_DIVISOR)
#define SDR_NBFM_RATE (BOARD_AUDIO_SAMPLE_RATE_HZ / SDR_NBFM_SAMPLE_RATE_DIVISOR)

#define SDR_NBFM_FILTER_NUM_STAGES SDR_IIR_STAGES_5

#define SDR_NBFM_FIR_CHAN_TAPS SDR_NBFM_LPF_2K5_NUM_TAPS
#define SDR_NBFM_FIR_CHAN_STATE_SIZE (SDR_NBFM_BLOCK_SIZE + SDR_NBFM_FIR_CHAN_TAPS - 1)

/* Audio passband edge (equivalent noise bandwidth) of the post-demodulator
 * LPF: the NBFM voice maximum, common to every band. */
#define SDR_NBFM_AUDIO_ENBW_HZ 3000u

#define SDR_NBFM_PHASE_TURNS_THRESHOLD 1000

#define SDR_NBFM_DC_BLOCK_FREQ (40.0f)

#define SDR_NBFM_SQUELCH_NOISE_DB (11.0f)

/* Sub-audible HPF: CTCSS tops out at 254.1 Hz. */
#define SDR_NBFM_TONE_HPF_HZ     300u
#define SDR_NBFM_TONE_HPF_STAGES SDR_IIR_STAGES_5

_Static_assert(SDR_NBFM_LPF_1K_NUM_TAPS == SDR_NBFM_FIR_CHAN_TAPS &&
               SDR_NBFM_LPF_5K_NUM_TAPS == SDR_NBFM_FIR_CHAN_TAPS,
               "NBFM channel LPFs must share the tap count");

typedef struct {
    float b0, b1;
    float a1;
    float x1, y1;
} FirstOrderIIR;

/* Channel bandwidths selectable by the host, widest first. The channel LPF and
 * the deviation move together; 1 kHz drops pre/de-emphasis (flat audio). */
typedef enum {
    SDR_NBFM_BAND_15K = 0,
    SDR_NBFM_BAND_10K,
    SDR_NBFM_BAND_7K,
    SDR_NBFM_BAND_COUNT
} SdrNbfmBand;

typedef struct {
    uint32_t     bandwidth_hz;
    uint32_t     deviation_hz;
    const float *chan_coeffs;
    bool         emphasis;
    float        signal_metric_ref;
} SdrNbfmBandCfg;

static const SdrNbfmBandCfg k_band_cfg[SDR_NBFM_BAND_COUNT] = {
    [SDR_NBFM_BAND_15K] = { 15000u, 5000u, sdr_nbfm_lpf_5k_coeffs,  true,  -3.6f },
    [SDR_NBFM_BAND_10K] = { 10000u, 2500u, sdr_nbfm_lpf_2k5_coeffs, true,  0.0f },
    [SDR_NBFM_BAND_7K]  = {  7000u, 1000u, sdr_nbfm_lpf_1k_coeffs,  false, 6.0f },
};

static FirstOrderIIR pre = SDR_NBFM_PRE_IIR_INIT;
static FirstOrderIIR de  = SDR_NBFM_DE_IIR_INIT;

static Sdr_DC_Filter s_dc;

static arm_fir_instance_f32 s_fir_chan[SDR_NBFM_BAND_COUNT][2];
static float s_fir_chan_state[SDR_NBFM_BAND_COUNT][2][SDR_NBFM_FIR_CHAN_STATE_SIZE];

static Sdr_FmDisc s_disc;
static float s_fir_diff_state[SDR_NBFM_BLOCK_SIZE + SDR_NBFM_FIR_NUM_TAPS - 1u];

static arm_biquad_casd_df1_inst_f32 s_filter_audio_inst;
static float s_filter_audio_coeffs[SDR_NBFM_FILTER_NUM_STAGES * 5];
static float s_filter_audio_state[SDR_NBFM_FILTER_NUM_STAGES * 4];

static float s_tx_angles[SDR_NBFM_BLOCK_SIZE];
static float s_tx_ctcss[SDR_NBFM_BLOCK_SIZE];
static float s_tx_audio[SDR_NBFM_BLOCK_SIZE];

/* Voice path only: TX while CTCSS is on, RX unless switched off. One design,
 * separate states. */
static float s_tone_hpf_coeffs[SDR_NBFM_TONE_HPF_STAGES * 5];
static arm_biquad_casd_df1_inst_f32 s_tone_hpf_tx;
static float s_tone_hpf_tx_state[SDR_NBFM_TONE_HPF_STAGES * 4];
static arm_biquad_casd_df1_inst_f32 s_tone_hpf_rx;
static float s_tone_hpf_rx_state[SDR_NBFM_TONE_HPF_STAGES * 4];

/* Squelch metric: HPF 3.5 kHz on the k-scaled discriminator output, before
 * emphasis and DC block. Updated once per demod block, read in the same ISR
 * pass. */
static arm_biquad_casd_df1_inst_f32 s_metric_hpf;
static float s_metric_hpf_state[SDR_SQUELCH_FM_HPF_STAGES * 4];
static float s_metric_buf[SDR_NBFM_BLOCK_SIZE];
static float s_signal_metric;

static struct {
    AppRadioMode radio_mode;
    SdrModulation rx_mode;
    SdrNbfmBand band;
    SdrNbfmAudio audio;
    bool rx_tone_filter;
} s_state;

static struct {
    bool reset_tx;
    bool reset_rx;
    bool reset_metric;
} s_pending;


static float first_order_iir(FirstOrderIIR *f, float x) {
    float y = f->b0 * x + f->b1 * f->x1 - f->a1 * f->y1;
    f->x1 = x;
    f->y1 = y;
    return y;
}

static void design_audio_filter(void) {
    SdrIirSpec spec = {
        .edge = SDR_IIR_EDGE_NOISE_BW,
        .f_edge_hz = (float)SDR_NBFM_AUDIO_ENBW_HZ,
        .fs_hz = (float)SDR_NBFM_RATE,
        .stages = SDR_NBFM_FILTER_NUM_STAGES,
        .type = SDR_IIR_TYPE_LPF
    };
    uint8_t stages;
    int res = Sdr_Iir_DesignEllip(&spec, s_filter_audio_coeffs, &stages);
    LOG_DEBUG("NBFM audio filter: enbw %u, stages: %u, res: %i",
              (unsigned)SDR_NBFM_AUDIO_ENBW_HZ, stages, res);
}

static void design_tone_hpf(void) {
    SdrIirSpec spec = {
        .edge = SDR_IIR_EDGE_NOISE_BW,
        .f_edge_hz = (float)SDR_NBFM_TONE_HPF_HZ,
        .fs_hz = (float)SDR_NBFM_RATE,
        .stages = SDR_NBFM_TONE_HPF_STAGES,
        .type = SDR_IIR_TYPE_HPF
    };
    uint8_t stages;
    int res = Sdr_Iir_DesignEllip(&spec, s_tone_hpf_coeffs, &stages);
    if (res != SDR_IIR_OK) {
        LOG_ERROR("NBFM tone HPF design failed: %i", res);
    }
}

/* Events subscriptions */

static void on_rx_tx(const AppEventRxTxEvent *evt, void *user) {
    (void)user;
    if (s_state.radio_mode != evt->mode) {
        uint32_t key = Bsp_Irq_SaveLow();
        if (s_state.radio_mode == APP_RADIO_MODE_TX) {
            s_pending.reset_tx = true;
        } else if (s_state.radio_mode == APP_RADIO_MODE_RX) {
            s_pending.reset_rx = true;
        }
        s_pending.reset_metric = true;
        Bsp_Irq_RestoreLow(key);
    }
    s_state.radio_mode = evt->mode;
}

/* Nearest of the three NBFM bands; the midpoints are the switch points. */
static SdrNbfmBand band_from_width(int32_t width) {
    if (width >= 12500) {
        return SDR_NBFM_BAND_15K;
    }
    if (width >= 8500) {
        return SDR_NBFM_BAND_10K;
    }
    return SDR_NBFM_BAND_7K;
}

/* The host delivers the band as the filter edges; only the width matters. */
static void nbfm_apply_band(const SdrRxCfg *cfg) {
    int32_t width = cfg->filter_high_hz - cfg->filter_low_hz;
    SdrNbfmBand band;

    if (width < 0) {
        width = -width;
    }
    if (width < 6000) {
        /* SSB-like edges, not a centred NBFM band: keep the current band. */
        return;
    }
    band = band_from_width(width);
    if (band == s_state.band) {
        return;
    }

    uint32_t key = Bsp_Irq_SaveLow();
    s_state.band = band;
    s_pending.reset_rx = true;
    Bsp_Irq_RestoreLow(key);
}

static void on_rx_cfg(const SdrRxCfg *cfg, void *user) {
    (void)user;
    /* The band is defined only for NFM; other modes keep the last NBFM band. */
    if (cfg->rx_mode == SDR_MOD_NFM) {
        nbfm_apply_band(cfg);
    }
    if (cfg->rx_mode == s_state.rx_mode) {
        return;
    }
    bool old_nfm = s_state.rx_mode == SDR_MOD_NFM;
    bool new_nfm = cfg->rx_mode == SDR_MOD_NFM;

    if (old_nfm != new_nfm) {
        uint32_t key = Bsp_Irq_SaveLow();
        s_pending.reset_metric = true;
        Bsp_Irq_RestoreLow(key);
    }
    s_state.rx_mode = cfg->rx_mode;
}

void Sdr_NBFM_Init(void) {
    for (uint32_t b = 0; b < (uint32_t)SDR_NBFM_BAND_COUNT; b++) {
        for (size_t i = 0; i < 2; i++) {
            arm_fir_init_f32(&s_fir_chan[b][i], SDR_NBFM_FIR_CHAN_TAPS,
                             k_band_cfg[b].chan_coeffs, s_fir_chan_state[b][i],
                             SDR_NBFM_BLOCK_SIZE);
        }
    }

    Sdr_FmDisc_Init(&s_disc, sdr_nbfm_fir_coeffs, SDR_NBFM_FIR_NUM_TAPS,
                    s_fir_diff_state, SDR_NBFM_BLOCK_SIZE,
                    (float)SDR_NBFM_PHASE_TURNS_THRESHOLD * (float)M_TWOPI);

    arm_biquad_cascade_df1_init_f32(&s_filter_audio_inst, SDR_NBFM_FILTER_NUM_STAGES,
                                    s_filter_audio_coeffs, s_filter_audio_state);

    arm_biquad_cascade_df1_init_f32(&s_metric_hpf, SDR_SQUELCH_FM_HPF_STAGES,
                                    sdr_squelch_fm_hpf_coeffs, s_metric_hpf_state);
    s_signal_metric = 0.0f;

    design_audio_filter();
    design_tone_hpf();
    arm_biquad_cascade_df1_init_f32(&s_tone_hpf_tx, SDR_NBFM_TONE_HPF_STAGES,
                                    s_tone_hpf_coeffs, s_tone_hpf_tx_state);
    arm_biquad_cascade_df1_init_f32(&s_tone_hpf_rx, SDR_NBFM_TONE_HPF_STAGES,
                                    s_tone_hpf_coeffs, s_tone_hpf_rx_state);
    Sdr_Ctcss_Init(SDR_NBFM_RATE);
    s_state.audio = SDR_NBFM_AUDIO_VOICE;
    s_state.rx_tone_filter = true;

    s_dc.alpha = (float)M_TWOPI * SDR_NBFM_DC_BLOCK_FREQ / SDR_NBFM_RATE;
    s_state.band = SDR_NBFM_BAND_10K;
    s_pending.reset_rx = true;
    s_pending.reset_tx = true;
    s_pending.reset_metric = true;

    if (App_Event_SubscribeRxTx(on_rx_tx, NULL)) {
        LOG_ERROR("App_Event_SubscribeRxTx failed");
    }
    if (App_Event_SubscribeRxCfg(on_rx_cfg, NULL) != 0) {
        LOG_ERROR("App_Event_SubscribeRxCfg failed");
    }
}

static void apply_pending(void) {
    SdrNbfmBand band = s_state.band;

    if (s_pending.reset_rx) {
        s_pending.reset_rx = false;
        de.x1 = 0.0f;
        de.y1 = 0.0f;
        Sdr_DC_Filter_Reset(&s_dc);
        Sdr_FmDisc_Reset(&s_disc);
        arm_fill_f32(0.0f, s_fir_chan_state[band][0], SDR_NBFM_FIR_CHAN_STATE_SIZE);
        arm_fill_f32(0.0f, s_fir_chan_state[band][1], SDR_NBFM_FIR_CHAN_STATE_SIZE);
        arm_fill_f32(0.0f, s_filter_audio_state, SDR_NBFM_FILTER_NUM_STAGES * 4);
        arm_fill_f32(0.0f, s_tone_hpf_rx_state, SDR_NBFM_TONE_HPF_STAGES * 4);
    }
    if (s_pending.reset_tx) {
        s_pending.reset_tx = false;
        pre.x1 = 0.0f;
        pre.y1 = 0.0f;
        s_tx_angles[SDR_NBFM_BLOCK_SIZE - 1] = 0.0f;
        arm_fill_f32(0.0f, s_tone_hpf_tx_state, SDR_NBFM_TONE_HPF_STAGES * 4);
        Sdr_Ctcss_Reset();
    }
    if (s_pending.reset_metric) {
        s_pending.reset_metric = false;
        arm_fill_f32(0.0f, s_metric_hpf_state, SDR_SQUELCH_FM_HPF_STAGES * 4);
    }
}

/* Squelch metric: the HPF-band (above voice) level of the discriminator output
 * relative to the calibrated reference. Noise invalid -> 0 (no data). */
static void nbfm_update_metric(uint32_t size) {
    if (!Sdr_Spectrum_Is_Noise_Valid()) {
        s_signal_metric = 0.0f;
        return;
    }

    SdrNbfmBand band = s_state.band;
    const SdrNbfmBandCfg *bc = &k_band_cfg[band];

    arm_biquad_cascade_df1_f32(&s_metric_hpf, s_metric_buf, s_metric_buf, size);
    float mean_power;
    arm_power_f32(s_metric_buf, size, &mean_power);
    s_signal_metric = SDR_NBFM_SQUELCH_NOISE_DB - Sdr_Pow2Db(mean_power) + bc->signal_metric_ref;
}

void Sdr_NBFM_Demod(const Sdr_IQ_Data *data, float *audio) {
    if (!data || !data->size || !audio || data->size != SDR_NBFM_BLOCK_SIZE) {
        return;
    }
    apply_pending();
    SdrNbfmBand band = s_state.band;
    const SdrNbfmBandCfg *bc = &k_band_cfg[band];
    bool voice = s_state.audio == SDR_NBFM_AUDIO_VOICE;
    bool emphasis = bc->emphasis && voice;

    /* LPF for signal */
    arm_fir_f32(&s_fir_chan[band][0], data->i, data->i, data->size);
    arm_fir_f32(&s_fir_chan[band][1], data->q, data->q, data->size);
    Sdr_FmDisc_Process(&s_disc, data->i, data->q, audio, data->size);
    float k = (float)SDR_NBFM_RATE / (float)bc->deviation_hz;
    for (size_t i = 0; i < data->size; i++) {
        float v = audio[i];
        /* Metric tap: k-scaled discriminator output before emphasis/DC. */
        s_metric_buf[i] = v * k;
        if (emphasis) {
            v = first_order_iir(&de, v);
        }
        v = Sdr_DC_Filter_Apply(&s_dc, v);
        audio[i] = v * k;
    }
    nbfm_update_metric(data->size);
    if (voice && s_state.rx_tone_filter) {
        arm_biquad_cascade_df1_f32(&s_tone_hpf_rx, audio, audio, data->size);
    }
    arm_biquad_cascade_df1_f32(&s_filter_audio_inst, audio, audio, data->size);
}

float Sdr_NBFM_GetSignalMetric(void) {
    return s_signal_metric;
}

void Sdr_NBFM_Mod(const float *audio, Sdr_IQ_Data *data) {
    if (!data || !data->size || !audio || data->size != SDR_NBFM_BLOCK_SIZE) {
        return;
    }
    apply_pending();
    const SdrNbfmBandCfg *bc = &k_band_cfg[s_state.band];
    float k = (float)bc->deviation_hz * 2.0f * (float)M_PI / (float)SDR_NBFM_RATE;
    float last_phase = s_tx_angles[SDR_NBFM_BLOCK_SIZE - 1];
    bool voice = s_state.audio == SDR_NBFM_AUDIO_VOICE;
    bool emphasis = bc->emphasis && voice;
    /* CTCSS goes in after pre-emphasis: its deviation is set directly. */
    bool ctcss = Sdr_Ctcss_Fill(s_tx_ctcss, data->size);
    if (ctcss && voice) {
        /* Keep voice out of the tone band. */
        arm_biquad_cascade_df1_f32(&s_tone_hpf_tx, audio, s_tx_audio, data->size);
        audio = s_tx_audio;
    }
    for (size_t i = 0; i < data->size; i++) {
        float v = audio[i];
        if (emphasis) {
            v = first_order_iir(&pre, v);
        }
        if (ctcss) {
            v += s_tx_ctcss[i];
        }
        last_phase += k * v;
        s_tx_angles[i] = last_phase;
    }
    Bsp_Cordic_SinCos(s_tx_angles, data->q, data->i, data->size);
}

void Sdr_NBFM_SetAudio(SdrNbfmAudio path) {
    if (path != SDR_NBFM_AUDIO_VOICE && path != SDR_NBFM_AUDIO_FLAT) {
        return;
    }
    uint32_t key = Bsp_Irq_SaveLow();
    if (s_state.audio != path) {
        s_state.audio = path;
        /* Emphasis and tone-HPF state belong to the old path. */
        s_pending.reset_rx = true;
        s_pending.reset_tx = true;
    }
    Bsp_Irq_RestoreLow(key);
    LOG_DEBUG("NBFM audio %s", path == SDR_NBFM_AUDIO_FLAT ? "flat" : "voice");
}

void Sdr_NBFM_SetRxToneFilter(bool on) {
    uint32_t key = Bsp_Irq_SaveLow();
    if (s_state.rx_tone_filter != on) {
        s_state.rx_tone_filter = on;
        s_pending.reset_rx = true;
    }
    Bsp_Irq_RestoreLow(key);
}

bool Sdr_NBFM_IsFlat(void) {
    return s_state.audio == SDR_NBFM_AUDIO_FLAT;
}
