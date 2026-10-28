/*
 * N32G4FR undocumented system control block at 0x40001800
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

#ifndef HW_N32G4FR_SYSCTL_H
#define HW_N32G4FR_SYSCTL_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

/*
 * The memory map shows 0x40001800-0x40001BFF as reserved; the only
 * documented register is the USB DP pull-up control (User Manual V3.5.0,
 * USB chapter, initialization note). The register name is ours.
 */
#define N32G4FR_SYSCTL_USB_PU   0x20

#define N32G4FR_SYSCTL_PERIPHERAL_SIZE 0x400
#define N32G4FR_SYSCTL_NREGS (N32G4FR_SYSCTL_PERIPHERAL_SIZE >> 2)

#define TYPE_N32G4FR_SYSCTL "n32g4fr.sysctl"
OBJECT_DECLARE_SIMPLE_TYPE(N32G4FRSysctlState, N32G4FR_SYSCTL)

struct N32G4FRSysctlState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    uint32_t regs[N32G4FR_SYSCTL_NREGS];

    /* Bit 28 of USB_PU: pull-up on USB DP, i.e. the device is attached */
    qemu_irq usb_dp_pullup;
};

#endif /* HW_N32G4FR_SYSCTL_H */
