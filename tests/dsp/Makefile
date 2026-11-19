# Host-side DSP self-tests for App/SDR. No hardware, no HAL: pure DSP sources
# are compiled for the build machine together with the portable C path of the
# CMSIS-DSP the firmware links (Middlewares/Third_Party/ARM_CMSIS).
#
#   make -C tests/dsp run

CC      ?= gcc
CFLAGS  ?= -std=gnu11 -Wall -Wextra -Wno-unused-parameter -O2
ROOT     = ../..
SDR      = $(ROOT)/App/SDR
DSP      = $(ROOT)/Middlewares/Third_Party/ARM_CMSIS
FILT     = $(DSP)/Source/FilteringFunctions
SUPP     = $(DSP)/Source/SupportFunctions
INCLUDES = -Istubs -I$(ROOT)/App -I$(ROOT)/Platform -I$(SDR) -I$(DSP)/Include -I$(DSP)/PrivateInclude \
           -I$(ROOT)/Devices -I$(ROOT)/Drivers/CMSIS/Include -I$(ROOT)/Drivers/CMSIS/Core/Include
LDLIBS   = -lm
# newlib (arm-none-eabi) math.h provides M_TWOPI; glibc does not.
INCLUDES += -DM_TWOPI=6.28318530717958647692

CMSIS_IIR = $(FILT)/arm_biquad_cascade_df1_f32.c $(FILT)/arm_biquad_cascade_df1_init_f32.c
CMSIS_FIR = $(FILT)/arm_fir_decimate_f32.c $(FILT)/arm_fir_decimate_init_f32.c
CMSIS_SUP = $(SUPP)/arm_copy_f32.c $(SUPP)/arm_fill_f32.c
CMSIS_BAS = $(DSP)/Source/BasicMathFunctions/arm_scale_f32.c
CMSIS_SIN = $(DSP)/Source/FastMathFunctions/arm_sin_f32.c $(DSP)/Source/CommonTables/arm_common_tables.c
CMSIS_FIRF = $(FILT)/arm_fir_f32.c $(FILT)/arm_fir_init_f32.c
CMSIS_PWR = $(DSP)/Source/StatisticsFunctions/arm_power_f32.c $(DSP)/Source/BasicMathFunctions/arm_offset_f32.c

IIR_SRCS    = test_sdr_iir.c $(SDR)/sdr_iir.c $(CMSIS_IIR)
BLOCKS_SRCS = test_sdr_blocks.c $(SDR)/sdr_halfband_decim.c $(SDR)/sdr_iir_decim.c \
              $(SDR)/sdr_soft_clipper.c $(CMSIS_IIR) $(CMSIS_FIR) $(CMSIS_SUP)

SIDEBAND_SRCS = test_sdr_sideband.c $(SDR)/sdr_sideband.c $(CMSIS_SUP)

COMP_SRCS = test_sdr_compressor.c $(SDR)/sdr_compressor.c stubs/stubs.c

AGC_SRCS = test_sdr_agc.c $(SDR)/sdr_agc.c stubs/stubs.c $(CMSIS_SUP) $(CMSIS_BAS)

CTCSS_SRCS = test_sdr_ctcss.c $(SDR)/sdr_ctcss.c $(SDR)/sdr_iir.c stubs/stubs.c $(CMSIS_IIR) $(CMSIS_SIN)

NBFM_CTCSS_SRCS = test_sdr_nbfm_ctcss.c $(SDR)/sdr_nbfm.c $(SDR)/sdr_ctcss.c $(SDR)/sdr_fm_disc.c \
                  $(SDR)/sdr_iir.c stubs/stubs.c $(CMSIS_IIR) $(CMSIS_SIN) $(CMSIS_FIRF) $(CMSIS_PWR) $(CMSIS_SUP)

BINS = test_sdr_iir test_sdr_blocks test_sdr_sideband test_sdr_compressor test_sdr_agc \
       test_sdr_ctcss test_sdr_nbfm_ctcss

.PHONY: all run clean

all: $(BINS)

test_sdr_iir: $(IIR_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(IIR_SRCS) $(LDLIBS)

test_sdr_blocks: $(BLOCKS_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(BLOCKS_SRCS) $(LDLIBS)

test_sdr_sideband: $(SIDEBAND_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(SIDEBAND_SRCS) $(LDLIBS)

test_sdr_compressor: $(COMP_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(COMP_SRCS) $(LDLIBS)

test_sdr_agc: $(AGC_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(AGC_SRCS) $(LDLIBS)

test_sdr_ctcss: $(CTCSS_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(CTCSS_SRCS) $(LDLIBS)

test_sdr_nbfm_ctcss: $(NBFM_CTCSS_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(NBFM_CTCSS_SRCS) $(LDLIBS)

run: $(BINS)
	./test_sdr_iir
	./test_sdr_blocks
	./test_sdr_sideband
	./test_sdr_compressor
	./test_sdr_agc
	./test_sdr_ctcss
	./test_sdr_nbfm_ctcss

clean:
	rm -f $(BINS)
