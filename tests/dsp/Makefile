# Host-side DSP self-tests for App/SDR. No hardware, no HAL: pure DSP sources
# are compiled for the build machine together with the portable C path of the
# CMSIS-DSP sources already vendored in Drivers/CMSIS/DSP.
#
#   make -C tests/dsp run

CC      ?= gcc
CFLAGS  ?= -std=gnu11 -Wall -Wextra -Wno-unused-parameter -O2
ROOT     = ../..
SDR      = $(ROOT)/App/SDR
DSP      = $(ROOT)/Drivers/CMSIS/DSP
FILT     = $(DSP)/Source/FilteringFunctions
SUPP     = $(DSP)/Source/SupportFunctions
INCLUDES = -I$(SDR) -I$(DSP)/Include -I$(DSP)/PrivateInclude \
           -I$(ROOT)/Devices -I$(ROOT)/Drivers/CMSIS/Include -I$(ROOT)/Drivers/CMSIS/Core/Include
LDLIBS   = -lm

CMSIS_IIR = $(FILT)/arm_biquad_cascade_df1_f32.c $(FILT)/arm_biquad_cascade_df1_init_f32.c
CMSIS_FIR = $(FILT)/arm_fir_decimate_f32.c $(FILT)/arm_fir_decimate_init_f32.c
CMSIS_SUP = $(SUPP)/arm_copy_f32.c $(SUPP)/arm_fill_f32.c

IIR_SRCS    = test_sdr_iir.c $(SDR)/sdr_iir.c $(CMSIS_IIR)
BLOCKS_SRCS = test_sdr_blocks.c $(SDR)/sdr_halfband_decim.c $(SDR)/sdr_iir_decim.c \
              $(SDR)/sdr_soft_clipper.c $(CMSIS_IIR) $(CMSIS_FIR) $(CMSIS_SUP)

SIDEBAND_SRCS = test_sdr_sideband.c $(SDR)/sdr_sideband.c $(CMSIS_SUP)

BINS = test_sdr_iir test_sdr_blocks test_sdr_sideband

.PHONY: all run clean

all: $(BINS)

test_sdr_iir: $(IIR_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(IIR_SRCS) $(LDLIBS)

test_sdr_blocks: $(BLOCKS_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(BLOCKS_SRCS) $(LDLIBS)

test_sdr_sideband: $(SIDEBAND_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(SIDEBAND_SRCS) $(LDLIBS)

run: $(BINS)
	./test_sdr_iir
	./test_sdr_blocks
	./test_sdr_sideband

clean:
	rm -f $(BINS)
