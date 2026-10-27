/*
 * N32G4FR ADC (minimal)
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

#ifndef HW_N32G4FR_ADC_H
#define HW_N32G4FR_ADC_H

#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define N32G4FR_ADC_NREGS ((0x5C >> 2) + 1)

/* Named GPIO output carrying the DMA request */
#define N32G4FR_ADC_DMA "dma"

#define TYPE_N32G4FR_ADC "n32g4fr.adc"
OBJECT_DECLARE_SIMPLE_TYPE(N32G4FRAdcState, N32G4FR_ADC)

struct N32G4FRAdcState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    uint32_t regs[N32G4FR_ADC_NREGS];
    /* Position in the regular sequence; < 0 when no sequence is running */
    int32_t seq_pos;
    QEMUTimer *conv_timer;

    /* Value returned for every conversion */
    uint32_t sample_value;

    qemu_irq irq;
    qemu_irq dma;
};

#endif /* HW_N32G4FR_ADC_H */
