# Host-side DSP tests for App/SDR

Compiles the real `App/SDR` DSP sources for the build machine (x86/ARM Linux,
plain gcc) together with the portable C path of the CMSIS-DSP the firmware
links (`Middlewares/Third_Party/ARM_CMSIS`). No hardware, no HAL, no emulator.

    make -C tests/dsp run

| Test | Block | Checks |
|---|---|---|
| `test_sdr_iir` | `sdr_iir.c` | SSB RX elliptic LPF/HPF design: passband flat, stopband ≥ 40 dB, at several edges |
| `test_sdr_blocks` | `sdr_halfband_decim.c`, `sdr_iir_decim.c`, `sdr_soft_clipper.c` | half-band output equals `arm_fir_decimate_f32` on the same taps; alias rejection; IIR decimator band edges; clipper continuity, slope, ceiling, monotonicity |
| `test_sdr_sideband` | `sdr_sideband.c` | RX opposite-sideband rejection; TX unwanted-sideband suppression |
| `test_sdr_compressor` | `sdr_compressor.c` | static curve: ratio above the knee, expansion below it, limiter at unity |
| `test_sdr_agc` | `sdr_agc.c` | static gain law vs. the noise-derived threshold; gain clamps; AGC-off gain |
| `test_sdr_ctcss` | `sdr_ctcss.c` | all 50 EIA/TIA tones + 150.0 Hz: frequency, level, residual; zero-phase start, phase-continuous change, stop at zero crossing, range checks. Info: TX voice HPF gain in the tone band at 150 and 300 Hz edges |
| `test_sdr_nbfm_ctcss` | `sdr_nbfm.c` + `sdr_ctcss.c` | end to end through `Sdr_NBFM_Mod`: tone deviation on the 15 kHz and 10 kHz bands, 1 kHz voice unchanged, clean stop. Info: tone level in RX audio via `Sdr_NBFM_Demod` loopback |
| `test_sdr_fm_audio` | `sdr_tx.c` + `sdr_nbfm.c` + `sdr_ctcss.c` | NBFM voice/flat audio path through `Sdr_Tx_Process` and `Sdr_NBFM_Demod`: voice HPF + compressor in voice, linear and flat in flat; CTCSS on both paths; 300 Hz TX tone HPF with CTCSS; RX tone filter on/off; flat RX low-end response |

`stubs/` replaces the few board services these blocks touch (`bsp_irq.h`
without the STM32 HAL, logging, event subscriptions, the spectrum noise
estimate, and the CORDIC via libm). `test_sdr_fm_audio` also stubs the
SSB/AM/CW modulators and resamplers, which NFM does not use. Tests are a measurement of current behaviour against limits read
from the source and the generated filter headers; they say nothing about CPU
time on the target.
