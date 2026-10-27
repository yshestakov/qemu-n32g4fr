/*
 * N32G4FR GPIO port
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

#ifndef HW_N32G4FR_GPIO_H
#define HW_N32G4FR_GPIO_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define N32G4FR_GPIO_NUM_PINS 16

#define TYPE_N32G4FR_GPIO "n32g4fr.gpio"
OBJECT_DECLARE_SIMPLE_TYPE(N32G4FRGpioState, N32G4FR_GPIO)

struct N32G4FRGpioState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    uint32_t pl_cfg;
    uint32_t ph_cfg;
    uint32_t pod;
    uint32_t plock;

    /* Levels driven onto the input lines by the board */
    uint32_t ext_level;
    /* Input lines the board has driven at least once */
    uint32_t ext_driven;
    /* Levels last driven on the output lines */
    uint32_t out_level;
    /* Pins that were outputs at the last update */
    uint32_t out_mask;

    /* Reset values, which differ per port */
    uint32_t pl_cfg_reset;
    uint32_t ph_cfg_reset;
    uint32_t pod_reset;

    qemu_irq out[N32G4FR_GPIO_NUM_PINS];
    /* QOM name, for tracing */
    char *name;
};

#endif /* HW_N32G4FR_GPIO_H */
