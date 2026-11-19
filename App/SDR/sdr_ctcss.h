#pragma once

#include <stdbool.h>
#include <stdint.h>

/* CTCSS (sub-audible tone squelch) encoder for NBFM TX.
 *
 * The tone is added inside the FM modulator after pre-emphasis, so the TX
 * voice HPF, compressor, clipper and emphasis never touch it and its
 * deviation is fixed: a fraction of the channel peak deviation. TIA-603
 * practice is about 12 %, i.e. +-600 Hz on a +-5 kHz channel and +-300 Hz on
 * a +-2.5 kHz one; the fraction keeps that ratio on every NBFM band.
 *
 * The frequency comes from a 32-bit phase accumulator: exact to fs / 2^32,
 * no drift, no CORDIC and no double-precision FPU needed (portable to F4).
 *
 * Sdr_Ctcss_Set may be called from task context; Sdr_Ctcss_Reset and
 * Sdr_Ctcss_Fill run in the modulator (DMA ISR) context. */

#define SDR_CTCSS_FREQ_MIN_DHZ          600u   /* 60.0 Hz */
#define SDR_CTCSS_FREQ_MAX_DHZ          2600u  /* 260.0 Hz */
#define SDR_CTCSS_DEV_DEFAULT_PERMILLE  120u   /* 12 % of peak deviation */
#define SDR_CTCSS_DEV_MAX_PERMILLE      250u

void Sdr_Ctcss_Init(uint32_t fs_hz);

/* freq_dhz: tone in 0.1 Hz units (885 = 88.5 Hz), 0 = off.
 * dev_permille: tone peak deviation in 1/1000 of the channel peak deviation,
 * 0 = default. Returns 0, or -1 when out of range (nothing changes).
 * A frequency change while the tone runs is phase-continuous; turning the
 * tone off ends it at the next zero crossing. */
int  Sdr_Ctcss_Set(uint16_t freq_dhz, uint16_t dev_permille);

/* Start of a transmission: the tone restarts at zero phase, so the carrier
 * starts without a deviation step. */
void Sdr_Ctcss_Reset(void);

/* Writes n tone samples, normalized so 1.0 is the channel peak deviation.
 * Returns false and leaves out untouched when no tone is running. */
bool Sdr_Ctcss_Fill(float *out, uint32_t n);
