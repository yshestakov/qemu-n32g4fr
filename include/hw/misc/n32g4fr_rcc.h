/*
 * N32G4FR RCC (clock ready/switch status is faked, nothing is clocked)
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

#ifndef HW_N32G4FR_RCC_H
#define HW_N32G4FR_RCC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

/* Register offsets, N32G4FR User Manual V3.5.0 Table 5-1 */
#define N32G4FR_RCC_CTRL        0x00
#define N32G4FR_RCC_CFG         0x04
#define N32G4FR_RCC_CLKINT      0x08
#define N32G4FR_RCC_APB2PRST    0x0C
#define N32G4FR_RCC_APB1PRST    0x10
#define N32G4FR_RCC_AHBPCLKEN   0x14
#define N32G4FR_RCC_APB2PCLKEN  0x18
#define N32G4FR_RCC_APB1PCLKEN  0x1C
#define N32G4FR_RCC_BDCTRL      0x20
#define N32G4FR_RCC_CTRLSTS     0x24
#define N32G4FR_RCC_AHBPRST     0x28
#define N32G4FR_RCC_CFG2        0x2C
#define N32G4FR_RCC_CFG3        0x30

#define N32G4FR_RCC_NREGS ((N32G4FR_RCC_CFG3 >> 2) + 1)
#define N32G4FR_RCC_PERIPHERAL_SIZE 0x400

#define TYPE_N32G4FR_RCC "n32g4fr.rcc"
OBJECT_DECLARE_SIMPLE_TYPE(N32G4FRRccState, N32G4FR_RCC)

struct N32G4FRRccState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    uint32_t regs[N32G4FR_RCC_NREGS];
};

#endif /* HW_N32G4FR_RCC_H */
