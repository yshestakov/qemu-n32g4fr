/*
 * N32G4FR USB full speed device controller, with a built-in CDC-ACM host
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

#ifndef HW_N32G4FR_USBD_H
#define HW_N32G4FR_USBD_H

#include "hw/core/sysbus.h"
#include "chardev/char-fe.h"
#include "qemu/fifo8.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define N32G4FR_USBD_NUM_EPS 8
#define N32G4FR_USBD_PMA_SIZE 512
/* Largest configuration descriptor the host reads */
#define N32G4FR_USBD_CTRL_BUF 256

#define TYPE_N32G4FR_USBD "n32g4fr.usbd"
OBJECT_DECLARE_SIMPLE_TYPE(N32G4FRUsbdState, N32G4FR_USBD)

struct N32G4FRUsbdState {
    SysBusDevice parent_obj;

    /* sysbus MMIO 0: registers, 1: packet buffer memory */
    MemoryRegion mmio;
    MemoryRegion pma_mmio;

    /* Device controller */
    uint32_t epr[N32G4FR_USBD_NUM_EPS];
    uint32_t ctrl;
    /* Latched USB_STS flags; CTRS/DIR/EP_ID are derived from the EPs */
    uint32_t sts;
    uint32_t fn;
    uint32_t addr;
    uint32_t buftab;
    uint8_t pma[N32G4FR_USBD_PMA_SIZE];

    /* Virtual host */
    QEMUTimer *frame_timer;
    int32_t host_state;
    int32_t host_wait;       /* frames left in the current wait */
    int32_t host_timeout;    /* frames before the current step fails */
    int32_t host_retries;
    int32_t enum_step;
    /* Current control transfer */
    uint8_t setup[8];
    uint8_t ctrl_buf[N32G4FR_USBD_CTRL_BUF];
    int32_t ctrl_len;        /* bytes expected (IN) or to send (OUT) */
    int32_t ctrl_done;       /* bytes transferred so far */
    int32_t ctrl_stage;
    /* What enumeration found */
    uint8_t dev_desc[18];
    uint16_t config_len;
    uint8_t config_value;
    uint8_t comm_iface;
    int8_t bulk_in;
    int8_t bulk_out;
    int8_t notify_in;

    CharFrontend chr;
    Fifo8 rx_fifo;           /* chardev -> bulk OUT */

    qemu_irq irq_lp;
    qemu_irq irq_hp;
};

#endif /* HW_N32G4FR_USBD_H */
