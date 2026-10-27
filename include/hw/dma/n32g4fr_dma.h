/*
 * N32G4FR DMA controller
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

#ifndef HW_N32G4FR_DMA_H
#define HW_N32G4FR_DMA_H

#include "hw/core/sysbus.h"
#include "system/memory.h"
#include "qom/object.h"

#define N32G4FR_DMA_NUM_CHANNELS 8
/*
 * Request inputs are numbered by their DMA_CHSELx.CH_SEL source number
 * (User Manual V3.5.0 section 8.6), e.g. DMA1 SPI2_RX = 21.
 */
#define N32G4FR_DMA_NUM_REQUESTS 64

/* DMA1 request sources used by the SoC */
#define N32G4FR_DMA1_REQ_ADC1       0
#define N32G4FR_DMA1_REQ_UART5_TX   1
#define N32G4FR_DMA1_REQ_USART3_TX  5
#define N32G4FR_DMA1_REQ_SPI1_RX    10
#define N32G4FR_DMA1_REQ_USART3_RX  11
#define N32G4FR_DMA1_REQ_SPI1_TX    15
#define N32G4FR_DMA1_REQ_USART1_TX  16
#define N32G4FR_DMA1_REQ_SPI2_RX    21
#define N32G4FR_DMA1_REQ_USART1_RX  23
#define N32G4FR_DMA1_REQ_SPI2_TX    25
#define N32G4FR_DMA1_REQ_USART2_RX  29
#define N32G4FR_DMA1_REQ_USART2_TX  34
#define N32G4FR_DMA1_REQ_ADC2       39
#define N32G4FR_DMA1_REQ_UART5_RX   40

/* DMA2 request sources used by the SoC */
#define N32G4FR_DMA2_REQ_SPI3_RX    4
#define N32G4FR_DMA2_REQ_UART6_RX   5
#define N32G4FR_DMA2_REQ_SPI3_TX    11
#define N32G4FR_DMA2_REQ_UART6_TX   12
#define N32G4FR_DMA2_REQ_UART4_RX   14
#define N32G4FR_DMA2_REQ_UART4_TX   24
#define N32G4FR_DMA2_REQ_UART7_RX   27
#define N32G4FR_DMA2_REQ_UART7_TX   30

#define TYPE_N32G4FR_DMA "n32g4fr.dma"
OBJECT_DECLARE_SIMPLE_TYPE(N32G4FRDmaState, N32G4FR_DMA)

typedef struct N32G4FRDmaChannel {
    uint32_t chcfg;
    uint32_t txnum;         /* remaining count, as read back */
    uint32_t txnum_reload;  /* programmed count, reloaded in circular mode */
    uint32_t paddr;
    uint32_t maddr;
    uint32_t chsel;
    uint32_t cur_paddr;
    uint32_t cur_maddr;
} N32G4FRDmaChannel;

struct N32G4FRDmaState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    MemoryRegion *downstream;
    AddressSpace downstream_as;
    QEMUBH *bh;

    /* 1 for DMA1, 2 for DMA2: selects the fixed request mapping */
    uint8_t controller;

    uint32_t intsts;
    uint32_t chmapen;
    N32G4FRDmaChannel ch[N32G4FR_DMA_NUM_CHANNELS];

    uint64_t drq;
    bool busy;

    qemu_irq irq[N32G4FR_DMA_NUM_CHANNELS];
};

#endif /* HW_N32G4FR_DMA_H */
