/*
 * N32G4FR USART / UART
 *
 * Based on the STM32F2XX USART model,
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

#ifndef HW_N32G4FR_USART_H
#define HW_N32G4FR_USART_H

#include "hw/core/sysbus.h"
#include "chardev/char-fe.h"
#include "qemu/timer.h"
#include "qom/object.h"

/* Named GPIO outputs carrying the DMA requests */
#define N32G4FR_USART_DMA_TX "dma-tx"
#define N32G4FR_USART_DMA_RX "dma-rx"

#define TYPE_N32G4FR_USART "n32g4fr.usart"
OBJECT_DECLARE_SIMPLE_TYPE(N32G4FRUsartState, N32G4FR_USART)

struct N32G4FRUsartState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    uint32_t sts;
    uint32_t dat;
    uint32_t brcf;
    uint32_t ctrl1;
    uint32_t ctrl2;
    uint32_t ctrl3;
    uint32_t gtp;

    /* STS was read since the last DAT read (IDLEF/OREF clear sequence) */
    bool sts_read;
    /* A character arrived since IDLEF was last set */
    bool idle_armed;
    QEMUTimer *idle_timer;

    /* Clock feeding the baud rate generator, for the frame time */
    uint32_t pclk_frequency;

    CharFrontend chr;
    qemu_irq irq;
    qemu_irq dma_tx;
    qemu_irq dma_rx;
};

#endif /* HW_N32G4FR_USART_H */
