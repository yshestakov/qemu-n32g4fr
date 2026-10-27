/*
 * N32G4FR embedded flash and flash interface
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

#ifndef HW_N32G4FR_FLASH_H
#define HW_N32G4FR_FLASH_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define N32G4FR_FLASH_PAGE_SIZE 2048

#define TYPE_N32G4FR_FLASH "n32g4fr.flash"
OBJECT_DECLARE_SIMPLE_TYPE(N32G4FRFlashState, N32G4FR_FLASH)

struct N32G4FRFlashState {
    SysBusDevice parent_obj;

    /* sysbus MMIO 0: flash interface registers, 1: the flash array */
    MemoryRegion mmio;
    MemoryRegion flash;
    uint8_t *storage;

    uint32_t ac;
    uint32_t sts;
    uint32_t ctrl;
    uint32_t add;
    uint32_t ob;
    uint32_t wrp;
    /* Position in the FLASH_KEY / FLASH_OPTKEY unlock sequences */
    uint8_t key_state;
    uint8_t optkey_state;
    /* A bad key sequence locks FLASH_CTRL until reset */
    bool key_fault;

    uint32_t flash_size;
    BlockBackend *blk;

    qemu_irq irq;
};

#endif /* HW_N32G4FR_FLASH_H */
