#pragma once

#include "sdr_common.h"

void Sdr_NBFM_Init(void);

/* FM audio path, for TX and RX together.
 *
 * VOICE (default): pre/de-emphasis on the bands that use it. On TX a 300 Hz
 * HPF keeps voice out of the CTCSS band while a tone is sent; on RX a 300 Hz
 * HPF removes received sub-audible tones unless Sdr_NBFM_SetRxToneFilter
 * turns it off.
 *
 * FLAT (data, USB audio): no emphasis and no sub-audible HPF either way, and
 * sdr_tx skips its voice HPF, compressor and clipper; its LPF still bounds
 * the occupied bandwidth. Tones and low-frequency data pass untouched.
 *
 * CTCSS (sdr_ctcss) is independent and works on either path. Setters are
 * task context. */
typedef enum {
    SDR_NBFM_AUDIO_VOICE = 0,
    SDR_NBFM_AUDIO_FLAT,
} SdrNbfmAudio;

void Sdr_NBFM_SetAudio(SdrNbfmAudio path);
void Sdr_NBFM_SetRxToneFilter(bool on);
bool Sdr_NBFM_IsFlat(void);
void Sdr_NBFM_Demod(const Sdr_IQ_Data *data, float *audio);
void Sdr_NBFM_Mod(const float *audio, Sdr_IQ_Data *data);

/* Squelch metric in dB, updated once per demod block. More is more
 * signal-like; may be negative. 0 means no data (noise estimate invalid). */
float Sdr_NBFM_GetSignalMetric(void);
