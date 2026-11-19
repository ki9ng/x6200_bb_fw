/* Host stub: no interrupts on the build machine, so the low-priority masking
 * used around shared DSP state is a no-op. Replaces Platform/bsp_irq.h, which
 * pulls in the STM32 HAL. */
#pragma once
#include <stdint.h>
static inline uint32_t Bsp_Irq_SaveLow(void) { return 0u; }
static inline void Bsp_Irq_RestoreLow(uint32_t key) { (void)key; }

/* The real header reaches these through stm32h7xx_hal.h; App/SDR relies on it. */
#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif
