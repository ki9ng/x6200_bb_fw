# Host-side DSP tests for App/SDR

Compiles the real `App/SDR` DSP sources for the build machine (x86/ARM Linux,
plain gcc) together with the portable C path of the CMSIS-DSP sources already
vendored in `Drivers/CMSIS/DSP`. No hardware, no HAL, no emulator.

    make -C tests/dsp run

| Test | Block | Checks |
|---|---|---|
| `test_sdr_iir` | `sdr_iir.c` | SSB RX elliptic LPF/HPF design: passband flat, stopband ≥ 40 dB, at several edges |
| `test_sdr_blocks` | `sdr_halfband_decim.c`, `sdr_iir_decim.c`, `sdr_soft_clipper.c` | half-band output equals `arm_fir_decimate_f32` on the same taps; alias rejection; IIR decimator band edges; clipper continuity, slope, ceiling, monotonicity |
| `test_sdr_sideband` | `sdr_sideband.c` | RX opposite-sideband rejection; TX unwanted-sideband suppression |
| `test_sdr_compressor` | `sdr_compressor.c` | static curve: ratio above the knee, expansion below it, limiter at unity |
| `test_sdr_agc` | `sdr_agc.c` | static gain law vs. the noise-derived threshold; gain clamps; AGC-off gain |

`stubs/` replaces the few board services these blocks touch (`bsp_irq.h`
without the STM32 HAL, logging, event subscriptions, the spectrum noise
estimate). Tests are a measurement of current behaviour against limits read
from the source and the generated filter headers; they say nothing about CPU
time on the target.
