/*
 * N32G4FR independent watchdog (IWDG)
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

#ifndef HW_N32G4FR_IWDG_H
#define HW_N32G4FR_IWDG_H

#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

/* Register offsets, N32G4FR User Manual V3.5.0 Table 16-2 */
#define N32G4FR_IWDG_KEY        0x00
#define N32G4FR_IWDG_PREDIV     0x04
#define N32G4FR_IWDG_RELV       0x08
#define N32G4FR_IWDG_STS        0x0C

#define N32G4FR_IWDG_PERIPHERAL_SIZE 0x400

#define TYPE_N32G4FR_IWDG "n32g4fr.iwdg"
OBJECT_DECLARE_SIMPLE_TYPE(N32G4FRIwdgState, N32G4FR_IWDG)

struct N32G4FRIwdgState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    QEMUTimer *timer;

    uint32_t prediv;
    uint32_t relv;
    /* Value the counter starts from: 0xFFF, or RELV after a reload */
    uint32_t count;
    bool unlocked;
    bool running;
    /* PWR_CTRL2.IWDGRSTEN: expiry resets the chip (default on) */
    bool reset_enable;

    /* Pulsed when the watchdog resets the chip (RCC IWDGRSTF) */
    qemu_irq reset_flag;
};

#endif /* HW_N32G4FR_IWDG_H */
