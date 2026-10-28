/*
 * N32G4FR alternate function I/O (AFIO)
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

#ifndef HW_N32G4FR_AFIO_H
#define HW_N32G4FR_AFIO_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

/* Register offsets, N32G4FR User Manual V3.5.0 Table 6-47 */
#define N32G4FR_AFIO_ECTRL      0x00
#define N32G4FR_AFIO_RMP_CFG    0x04
#define N32G4FR_AFIO_EXTI_CFG1  0x08
#define N32G4FR_AFIO_EXTI_CFG2  0x0C
#define N32G4FR_AFIO_EXTI_CFG3  0x10
#define N32G4FR_AFIO_EXTI_CFG4  0x14
#define N32G4FR_AFIO_RMP_CFG3   0x20
#define N32G4FR_AFIO_RMP_CFG4   0x24
#define N32G4FR_AFIO_RMP_CFG5   0x28

#define N32G4FR_AFIO_NREGS ((N32G4FR_AFIO_RMP_CFG5 >> 2) + 1)
#define N32G4FR_AFIO_PERIPHERAL_SIZE 0x400

#define TYPE_N32G4FR_AFIO "n32g4fr.afio"
OBJECT_DECLARE_SIMPLE_TYPE(N32G4FRAfioState, N32G4FR_AFIO)

struct N32G4FRAfioState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    uint32_t regs[N32G4FR_AFIO_NREGS];
};

#endif /* HW_N32G4FR_AFIO_H */
