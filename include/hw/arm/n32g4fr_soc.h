/*
 * Nationstech N32G4FR SoC
 *
 * Based on the STM32F405 SoC model,
 * Copyright (c) 2014 Alistair Francis <alistair@alistair23.me>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef HW_ARM_N32G4FR_SOC_H
#define HW_ARM_N32G4FR_SOC_H

#include "hw/misc/n32g4fr_rcc.h"
#include "hw/dma/n32g4fr_dma.h"
#include "hw/gpio/n32g4fr_gpio.h"
#include "hw/nvram/n32g4fr_flash.h"
#include "hw/usb/n32g4fr_usbd.h"
#include "hw/ssi/n32g4fr_spi.h"
#include "hw/char/n32g4fr_usart.h"
#include "hw/adc/n32g4fr_adc.h"
#include "hw/timer/stm32f2xx_timer.h"
#include "hw/core/or-irq.h"
#include "hw/arm/armv7m.h"
#include "qom/object.h"

#define TYPE_N32G4FR_SOC "n32g4fr-soc"
OBJECT_DECLARE_SIMPLE_TYPE(N32G4FRState, N32G4FR_SOC)

/* USART1-3 and UART4-7 */
#define N32_NUM_USARTS 7
/* TIM1 (update interrupt only) and TIM2-5 */
#define N32_NUM_TIMERS 5
#define N32_NUM_SPIS 3
#define N32_NUM_DMAS 2
#define N32_NUM_ADCS 2
/* GPIOA-E */
#define N32_NUM_GPIOS 5

/* N32G4FR User Manual V3.5.0 section 2.2 */
#define N32_FLASH_BASE_ADDRESS 0x08000000
#define N32_FLASH_SIZE (512 * 1024)
/* 128 KB SRAM followed contiguously by 16 KB R-SRAM */
#define N32_SRAM_BASE_ADDRESS 0x20000000
#define N32_SRAM_SIZE (144 * 1024)
/* ICode/DCode alias of the first 128 KB of SRAM */
#define N32_SRAM_ALIAS_BASE_ADDRESS 0x10000000
#define N32_SRAM_ALIAS_SIZE (128 * 1024)

/* System configuration area, UM V3.5.0 Table 2-2 and section 31.1 */
#define N32_SYSCFG_BASE_ADDRESS 0x1FFFF000
#define N32_SYSCFG_SIZE 0x800
#define N32_UCID_OFFSET 0x7C0
#define N32_UCID_SIZE 16
#define N32_UID_OFFSET 0x7F0
#define N32_UID_SIZE 12

struct N32G4FRState {
    SysBusDevice parent_obj;

    ARMv7MState armv7m;

    N32G4FRRccState rcc;
    N32G4FRFlashState flash;
    N32G4FRUsbdState usbd;
    N32G4FRDmaState dma[N32_NUM_DMAS];
    N32G4FRGpioState gpio[N32_NUM_GPIOS];
    N32G4FRUsartState usart[N32_NUM_USARTS];
    N32G4FRSpiState spi[N32_NUM_SPIS];
    STM32F2XXTimerState timer[N32_NUM_TIMERS];
    OrIRQState adc_irqs;
    N32G4FRAdcState adc[N32_NUM_ADCS];

    MemoryRegion sram;
    MemoryRegion sram_alias;
    MemoryRegion flash_alias;
    MemoryRegion syscfg;

    /* Hex strings for the UID/UCID in the system configuration area */
    char *uid;
    char *ucid;

    Clock *sysclk;
    Clock *refclk;
};

#endif
