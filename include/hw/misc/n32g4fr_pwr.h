/*
 * N32G4FR power control (PWR)
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

#ifndef HW_N32G4FR_PWR_H
#define HW_N32G4FR_PWR_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

/* Register offsets, N32G4FR User Manual V3.5.0 Table 3-3 */
#define N32G4FR_PWR_CTRL        0x00
#define N32G4FR_PWR_CTRLSTS     0x04
#define N32G4FR_PWR_CTRL2       0x08
#define N32G4FR_PWR_CTRL3       0x0C
/* Not in the user manual, only in N32G4FR.svd */
#define N32G4FR_PWR_CTRL4       0x10
#define N32G4FR_PWR_CTRL5       0x14
#define N32G4FR_PWR_CTRL6       0x18
#define N32G4FR_PWR_CTRL7       0x1C

#define N32G4FR_PWR_NREGS ((N32G4FR_PWR_CTRL7 >> 2) + 1)
#define N32G4FR_PWR_PERIPHERAL_SIZE 0x400

#define TYPE_N32G4FR_PWR "n32g4fr.pwr"
OBJECT_DECLARE_SIMPLE_TYPE(N32G4FRPwrState, N32G4FR_PWR)

struct N32G4FRPwrState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    uint32_t regs[N32G4FR_PWR_NREGS];

    /* PWR_CTRL2.IWDGRSTEN: the independent watchdog may reset the chip */
    qemu_irq iwdg_reset_en;
};

#endif /* HW_N32G4FR_PWR_H */
