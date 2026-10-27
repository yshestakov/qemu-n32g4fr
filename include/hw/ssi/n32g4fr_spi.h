/*
 * N32G4FR SPI controller (master mode)
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

#ifndef HW_N32G4FR_SPI_H
#define HW_N32G4FR_SPI_H

#include "hw/core/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qom/object.h"

/* Named GPIO outputs carrying the DMA requests */
#define N32G4FR_SPI_DMA_TX "dma-tx"
#define N32G4FR_SPI_DMA_RX "dma-rx"

#define TYPE_N32G4FR_SPI "n32g4fr.spi"
OBJECT_DECLARE_SIMPLE_TYPE(N32G4FRSpiState, N32G4FR_SPI)

struct N32G4FRSpiState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    uint32_t ctrl1;
    uint32_t ctrl2;
    uint32_t sts;
    uint32_t rx_dat;
    uint32_t crcpoly;
    uint32_t i2scfg;
    uint32_t i2sprediv;

    qemu_irq irq;
    qemu_irq dma_tx;
    qemu_irq dma_rx;
    SSIBus *ssi;
};

#endif /* HW_N32G4FR_SPI_H */
