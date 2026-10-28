/*
 * N32G4FR USB full speed device controller, with a built-in CDC-ACM host
 *
 * Device side (N32G4FR User Manual V3.5.0 chapter 19; it is the STM32F103
 * USB device block): eight endpoint registers, USB_CTRL/STS/FN/ADDR/BUFTAB,
 * and the 512 byte packet buffer memory the CPU sees as 16 bit words at a
 * 32 bit stride. Single buffered control, bulk and interrupt endpoints are
 * modelled; double buffering and isochronous endpoints are not.
 *
 * There is no real USB host on the other side, so the device contains a
 * minimal one, driven by a 1 ms frame timer. Once the firmware powers the
 * controller up and the DP pull-up is on (the "dp-pullup" input; on when
 * unconnected), it resets the bus, enumerates the device over endpoint 0
 * (device and configuration descriptors, SET_ADDRESS, SET_CONFIGURATION),
 * opens the first CDC-ACM function it finds (SET_LINE_CODING 115200 8N1,
 * SET_CONTROL_LINE_STATE with DTR and RTS) and then bridges its bulk
 * endpoints to the "chardev": data the firmware sends on bulk IN is written
 * to the chardev, chardev input is sent on bulk OUT. With e.g.
 * -chardev pty the firmware's virtual COM port appears as a host tty.
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

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/bitops.h"
#include "trace.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "hw/usb/n32g4fr_usbd.h"

/* Registers */
#define USB_EP0         0x00
#define USB_EP7         0x1C
#define USB_CTRL        0x40
#define USB_STS         0x44
#define USB_FN          0x48
#define USB_ADDR        0x4C
#define USB_BUFTAB      0x50

/* USB_EPn */
#define EP_EA_MASK      0x000F
#define EP_STS_TX_SHIFT 4
#define EP_DATTOG_TX    BIT(6)
#define EP_CTRS_TX      BIT(7)
#define EP_KIND         BIT(8)
#define EP_TYPE_SHIFT   9
#define EP_SETUP        BIT(11)
#define EP_STS_RX_SHIFT 12
#define EP_DATTOG_RX    BIT(14)
#define EP_CTRS_RX      BIT(15)
#define EP_RW_MASK      (EP_EA_MASK | EP_KIND | (3 << EP_TYPE_SHIFT))
#define EP_TOGGLE_MASK  0x7070
#define EP_CTRS_MASK    (EP_CTRS_RX | EP_CTRS_TX)

#define EP_TYPE_BULK    0
#define EP_TYPE_ISO     2

#define STAT_DISABLED   0
#define STAT_STALL      1
#define STAT_NAK        2
#define STAT_VALID      3

/* USB_CTRL */
#define CTRL_FRST       BIT(0)
#define CTRL_PD         BIT(1)
#define CTRL_RESET      (CTRL_FRST | CTRL_PD)
#define CTRL_MASK       0xFF1F

/* USB_STS */
#define STS_DIR         BIT(4)
#define STS_ESOF        BIT(8)
#define STS_SOF         BIT(9)
#define STS_RST         BIT(10)
#define STS_CTRS        BIT(15)
#define STS_LATCHED     0x7F00

/* USB_FN */
#define FN_FNUM_MASK    0x7FF
#define FN_RXDP_STS     BIT(15)

/* USB_ADDR */
#define ADDR_EFUC       BIT(7)
#define ADDR_ADD_MASK   0x7F

/* Buffer description table entry fields (16 bit words) */
#define BT_ADDR_TX      0
#define BT_COUNT_TX     1
#define BT_ADDR_RX      2
#define BT_COUNT_RX     3

#define FRAME_NS        (1 * SCALE_MS)

/* Virtual host */
enum {
    HOST_DETACHED,
    HOST_RESET_WAIT,    /* attached, bus reset after host_wait frames */
    HOST_ENABLE_WAIT,   /* reset done, waiting for USB_ADDR.EFUC */
    HOST_CONTROL,       /* enumerating */
    HOST_RUNNING,       /* bridging the CDC data endpoints */
    HOST_FAILED,
};

enum {
    STAGE_SETUP,
    STAGE_DATA_IN,
    STAGE_DATA_OUT,
    STAGE_STATUS_IN,
    STAGE_STATUS_OUT,
    STAGE_WAIT_ADDR,
    STAGE_DONE,
};

enum {
    ENUM_GET_DEVICE,
    ENUM_SET_ADDRESS,
    ENUM_GET_CONFIG_HEADER,
    ENUM_GET_CONFIG,
    ENUM_SET_CONFIG,
    ENUM_SET_LINE_CODING,
    ENUM_SET_CONTROL_LINE_STATE,
    ENUM_DONE,
};

#define HOST_ADDRESS        1
#define HOST_ATTACH_FRAMES  20
#define HOST_STEP_FRAMES    500
#define HOST_MAX_RETRIES    3
/* Packets per bulk direction per frame */
#define HOST_BULK_PACKETS   8
#define RX_FIFO_SIZE        1024

static const char *const host_state_name[] = {
    "detached", "reset-wait", "enable-wait", "control", "running", "failed",
};

/* Packet buffer memory, as seen by the USB module (byte addresses) */
static uint16_t pma_read16(N32G4FRUsbdState *s, uint32_t addr)
{
    addr &= N32G4FR_USBD_PMA_SIZE - 2;
    return s->pma[addr] | (s->pma[addr + 1] << 8);
}

static void pma_read(N32G4FRUsbdState *s, uint32_t addr, uint8_t *buf,
                     int len)
{
    for (int i = 0; i < len; i++) {
        buf[i] = s->pma[(addr + i) % N32G4FR_USBD_PMA_SIZE];
    }
}

static void pma_write(N32G4FRUsbdState *s, uint32_t addr,
                      const uint8_t *buf, int len)
{
    for (int i = 0; i < len; i++) {
        s->pma[(addr + i) % N32G4FR_USBD_PMA_SIZE] = buf[i];
    }
}

static uint32_t bt_addr(N32G4FRUsbdState *s, int ep, int field)
{
    return s->buftab + ep * 8 + field * 2;
}

static uint16_t bt_read(N32G4FRUsbdState *s, int ep, int field)
{
    return pma_read16(s, bt_addr(s, ep, field));
}

static void bt_write(N32G4FRUsbdState *s, int ep, int field, uint16_t value)
{
    uint32_t addr = bt_addr(s, ep, field) & (N32G4FR_USBD_PMA_SIZE - 2);

    s->pma[addr] = value;
    s->pma[addr + 1] = value >> 8;
}

/* Size of an endpoint's receive buffer, from COUNTn_RX.BL_SIZE/NUM_BLOCK */
static int rx_buffer_size(N32G4FRUsbdState *s, int ep)
{
    uint16_t count = bt_read(s, ep, BT_COUNT_RX);
    int blocks = extract32(count, 10, 5);

    return (count & BIT(15)) ? 32 * (blocks + 1) : 2 * blocks;
}

static void n32g4fr_usbd_update_irq(N32G4FRUsbdState *s)
{
    uint32_t sts = s->sts;

    for (int ep = 0; ep < N32G4FR_USBD_NUM_EPS; ep++) {
        if (s->epr[ep] & EP_CTRS_MASK) {
            sts |= STS_CTRS;
            break;
        }
    }
    qemu_set_irq(s->irq_lp, !!(sts & s->ctrl & 0xFF00));
}

static uint32_t n32g4fr_usbd_sts(N32G4FRUsbdState *s)
{
    uint32_t sts = s->sts;

    /* CTRS, DIR and EP_ID report the highest priority pending endpoint */
    for (int ep = 0; ep < N32G4FR_USBD_NUM_EPS; ep++) {
        if (s->epr[ep] & EP_CTRS_MASK) {
            sts |= STS_CTRS | ep;
            if (s->epr[ep] & EP_CTRS_RX) {
                sts |= STS_DIR;
            }
            break;
        }
    }
    return sts;
}

static int ep_stat_tx(N32G4FRUsbdState *s, int ep)
{
    return extract32(s->epr[ep], EP_STS_TX_SHIFT, 2);
}

static int ep_stat_rx(N32G4FRUsbdState *s, int ep)
{
    return extract32(s->epr[ep], EP_STS_RX_SHIFT, 2);
}

/*
 * Host side transactions. Each returns the number of bytes moved, -1 if the
 * endpoint NAKs (not ready), -2 if it is stalled or disabled.
 */
static int host_find_ep(N32G4FRUsbdState *s, int num)
{
    for (int ep = 0; ep < N32G4FR_USBD_NUM_EPS; ep++) {
        if ((s->epr[ep] & EP_EA_MASK) == num && (s->epr[ep] & 0x3030)) {
            return ep;
        }
    }
    return num == 0 ? 0 : -1;
}

static int host_setup(N32G4FRUsbdState *s, const uint8_t *setup)
{
    int ep = host_find_ep(s, 0);

    if (ep_stat_rx(s, ep) == STAT_DISABLED) {
        return -2;
    }
    if (s->epr[ep] & EP_CTRS_MASK) {
        /*
         * The firmware has not handled the previous transaction yet; a real
         * device would have long done so before the next SETUP arrives.
         */
        return -1;
    }
    pma_write(s, bt_read(s, ep, BT_ADDR_RX), setup, 8);
    bt_write(s, ep, BT_COUNT_RX, (bt_read(s, ep, BT_COUNT_RX) & 0xFC00) | 8);
    /* SETUP is always accepted; both directions NAK until the firmware acts */
    s->epr[ep] = deposit32(s->epr[ep], EP_STS_RX_SHIFT, 2, STAT_NAK);
    s->epr[ep] = deposit32(s->epr[ep], EP_STS_TX_SHIFT, 2, STAT_NAK);
    s->epr[ep] |= EP_CTRS_RX | EP_SETUP;
    s->epr[ep] ^= EP_DATTOG_RX;
    trace_n32g4fr_usbd_setup(setup[0], setup[1],
                             setup[2] | (setup[3] << 8),
                             setup[4] | (setup[5] << 8),
                             setup[6] | (setup[7] << 8));
    n32g4fr_usbd_update_irq(s);
    return 8;
}

static int host_in(N32G4FRUsbdState *s, int ep, uint8_t *buf, int max)
{
    int len;

    if (ep < 0) {
        return -2;
    }
    switch (ep_stat_tx(s, ep)) {
    case STAT_VALID:
        break;
    case STAT_NAK:
        return -1;
    default:
        return -2;
    }
    len = MIN(bt_read(s, ep, BT_COUNT_TX) & 0x3FF, max);
    pma_read(s, bt_read(s, ep, BT_ADDR_TX), buf, len);
    s->epr[ep] = deposit32(s->epr[ep], EP_STS_TX_SHIFT, 2, STAT_NAK);
    s->epr[ep] |= EP_CTRS_TX;
    s->epr[ep] ^= EP_DATTOG_TX;
    trace_n32g4fr_usbd_in(s->epr[ep] & EP_EA_MASK, len);
    n32g4fr_usbd_update_irq(s);
    return len;
}

static int host_out(N32G4FRUsbdState *s, int ep, const uint8_t *buf, int len)
{
    if (ep < 0) {
        return -2;
    }
    switch (ep_stat_rx(s, ep)) {
    case STAT_VALID:
        break;
    case STAT_NAK:
        return -1;
    default:
        return -2;
    }
    if (s->epr[ep] & EP_CTRS_RX) {
        return -1;
    }
    len = MIN(len, rx_buffer_size(s, ep));
    pma_write(s, bt_read(s, ep, BT_ADDR_RX), buf, len);
    bt_write(s, ep, BT_COUNT_RX, (bt_read(s, ep, BT_COUNT_RX) & 0xFC00) | len);
    s->epr[ep] = deposit32(s->epr[ep], EP_STS_RX_SHIFT, 2, STAT_NAK);
    s->epr[ep] &= ~EP_SETUP;
    s->epr[ep] |= EP_CTRS_RX;
    s->epr[ep] ^= EP_DATTOG_RX;
    trace_n32g4fr_usbd_out(s->epr[ep] & EP_EA_MASK, len);
    n32g4fr_usbd_update_irq(s);
    return len;
}

static void host_set_state(N32G4FRUsbdState *s, int state)
{
    if (s->host_state != state) {
        trace_n32g4fr_usbd_host_state(host_state_name[state]);
    }
    s->host_state = state;
}

static void host_bus_reset(N32G4FRUsbdState *s)
{
    s->sts |= STS_RST;
    s->addr = 0;
    s->enum_step = ENUM_GET_DEVICE;
    s->host_timeout = HOST_STEP_FRAMES;
    s->bulk_in = s->bulk_out = s->notify_in = -1;
    host_set_state(s, HOST_ENABLE_WAIT);
    n32g4fr_usbd_update_irq(s);
}

static void host_fail(N32G4FRUsbdState *s, const char *why)
{
    qemu_log_mask(LOG_GUEST_ERROR, "n32g4fr-usbd: enumeration step %d "
                  "stage %d: %s\n", s->enum_step, s->ctrl_stage, why);
    if (++s->host_retries > HOST_MAX_RETRIES) {
        qemu_log_mask(LOG_GUEST_ERROR, "n32g4fr-usbd: giving up\n");
        host_set_state(s, HOST_FAILED);
        return;
    }
    host_bus_reset(s);
}

static void host_start_request(N32G4FRUsbdState *s)
{
    uint8_t type = 0, req = 0;
    uint16_t value = 0, index = 0, len = 0;

    switch (s->enum_step) {
    case ENUM_GET_DEVICE:
        type = 0x80, req = 6, value = 0x0100, len = sizeof(s->dev_desc);
        break;
    case ENUM_SET_ADDRESS:
        type = 0x00, req = 5, value = HOST_ADDRESS;
        break;
    case ENUM_GET_CONFIG_HEADER:
        type = 0x80, req = 6, value = 0x0200, len = 9;
        break;
    case ENUM_GET_CONFIG:
        type = 0x80, req = 6, value = 0x0200, len = s->config_len;
        break;
    case ENUM_SET_CONFIG:
        type = 0x00, req = 9, value = s->config_value;
        break;
    case ENUM_SET_LINE_CODING:
        /* 115200 baud, 1 stop bit, no parity, 8 data bits */
        type = 0x21, req = 0x20, index = s->comm_iface, len = 7;
        stl_le_p(s->ctrl_buf, 115200);
        s->ctrl_buf[4] = 0;
        s->ctrl_buf[5] = 0;
        s->ctrl_buf[6] = 8;
        break;
    case ENUM_SET_CONTROL_LINE_STATE:
        /* DTR | RTS */
        type = 0x21, req = 0x22, value = 3, index = s->comm_iface;
        break;
    }

    s->setup[0] = type;
    s->setup[1] = req;
    stw_le_p(&s->setup[2], value);
    stw_le_p(&s->setup[4], index);
    stw_le_p(&s->setup[6], len);
    s->ctrl_len = len;
    s->ctrl_done = 0;
    s->ctrl_stage = STAGE_SETUP;
    s->host_timeout = HOST_STEP_FRAMES;
}

/* Find the CDC-ACM function in the configuration descriptor */
static bool host_parse_config(N32G4FRUsbdState *s)
{
    int iface_class = -1;
    bool have_comm = false;

    s->config_value = s->ctrl_buf[5];
    for (int pos = 0; pos + 2 <= s->ctrl_done; ) {
        uint8_t len = s->ctrl_buf[pos];
        uint8_t type = s->ctrl_buf[pos + 1];

        if (len < 2 || pos + len > s->ctrl_done) {
            break;
        }
        if (type == 4 && len >= 9) {
            /* Interface */
            iface_class = s->ctrl_buf[pos + 5];
            if (iface_class == 0x02 && !have_comm) {
                s->comm_iface = s->ctrl_buf[pos + 2];
                have_comm = true;
            }
        } else if (type == 5 && len >= 7) {
            /* Endpoint */
            uint8_t ep = s->ctrl_buf[pos + 2];
            uint8_t attr = s->ctrl_buf[pos + 3] & 3;

            if (iface_class == 0x0A && attr == 2) {
                if ((ep & 0x80) && s->bulk_in < 0) {
                    s->bulk_in = ep & 0xF;
                } else if (!(ep & 0x80) && s->bulk_out < 0) {
                    s->bulk_out = ep & 0xF;
                }
            } else if (iface_class == 0x02 && attr == 3 && (ep & 0x80)) {
                s->notify_in = ep & 0xF;
            }
        }
        pos += len;
    }

    trace_n32g4fr_usbd_cdc(s->comm_iface, s->bulk_in, s->bulk_out,
                           s->notify_in);
    return have_comm && s->bulk_in >= 0 && s->bulk_out >= 0;
}

/* Handle the result of a finished request and move to the next one */
static void host_request_done(N32G4FRUsbdState *s)
{
    switch (s->enum_step) {
    case ENUM_GET_DEVICE:
        memcpy(s->dev_desc, s->ctrl_buf, sizeof(s->dev_desc));
        break;
    case ENUM_GET_CONFIG_HEADER:
        s->config_len = MIN(lduw_le_p(&s->ctrl_buf[2]),
                            N32G4FR_USBD_CTRL_BUF);
        break;
    case ENUM_GET_CONFIG:
        if (!host_parse_config(s)) {
            qemu_log_mask(LOG_GUEST_ERROR, "n32g4fr-usbd: no CDC-ACM "
                          "function found\n");
            host_set_state(s, HOST_FAILED);
            return;
        }
        break;
    }

    if (++s->enum_step == ENUM_DONE) {
        s->host_retries = 0;
        host_set_state(s, HOST_RUNNING);
        qemu_chr_fe_accept_input(&s->chr);
        return;
    }
    host_start_request(s);
}

/* Advance the current control transfer; returns true on progress */
static bool host_control_step(N32G4FRUsbdState *s)
{
    int ep0 = host_find_ep(s, 0);
    int mps = s->dev_desc[7] ? s->dev_desc[7] : 8;
    uint8_t zlp[64];
    int n;

    switch (s->ctrl_stage) {
    case STAGE_SETUP:
        n = host_setup(s, s->setup);
        if (n < 0) {
            return false;
        }
        if (s->ctrl_len == 0) {
            s->ctrl_stage = STAGE_STATUS_IN;
        } else if (s->setup[0] & 0x80) {
            s->ctrl_stage = STAGE_DATA_IN;
        } else {
            s->ctrl_stage = STAGE_DATA_OUT;
        }
        return true;
    case STAGE_DATA_IN:
        n = host_in(s, ep0, s->ctrl_buf + s->ctrl_done,
                    s->ctrl_len - s->ctrl_done);
        if (n == -2) {
            host_fail(s, "stalled in data stage");
            return false;
        }
        if (n < 0) {
            return false;
        }
        s->ctrl_done += n;
        if (n < mps || s->ctrl_done >= s->ctrl_len) {
            s->ctrl_stage = STAGE_STATUS_OUT;
        }
        return true;
    case STAGE_DATA_OUT:
        n = host_out(s, ep0, s->ctrl_buf + s->ctrl_done,
                     MIN(mps, s->ctrl_len - s->ctrl_done));
        if (n == -2) {
            host_fail(s, "stalled in data stage");
            return false;
        }
        if (n < 0) {
            return false;
        }
        s->ctrl_done += n;
        if (s->ctrl_done >= s->ctrl_len) {
            s->ctrl_stage = STAGE_STATUS_IN;
        }
        return true;
    case STAGE_STATUS_IN:
        n = host_in(s, ep0, zlp, sizeof(zlp));
        if (n == -2) {
            host_fail(s, "stalled in status stage");
            return false;
        }
        if (n < 0) {
            return false;
        }
        s->ctrl_stage = s->enum_step == ENUM_SET_ADDRESS ?
                        STAGE_WAIT_ADDR : STAGE_DONE;
        return true;
    case STAGE_STATUS_OUT:
        n = host_out(s, ep0, NULL, 0);
        if (n == -2) {
            host_fail(s, "stalled in status stage");
            return false;
        }
        if (n < 0) {
            return false;
        }
        s->ctrl_stage = STAGE_DONE;
        return true;
    case STAGE_WAIT_ADDR:
        /* The firmware sets the new address after the status stage */
        if ((s->addr & ADDR_ADD_MASK) != HOST_ADDRESS) {
            return false;
        }
        s->ctrl_stage = STAGE_DONE;
        return true;
    case STAGE_DONE:
        host_request_done(s);
        return s->host_state == HOST_CONTROL;
    }
    return false;
}

static void host_bulk(N32G4FRUsbdState *s)
{
    uint8_t buf[64];
    int n;

    for (int i = 0; i < HOST_BULK_PACKETS; i++) {
        n = host_in(s, host_find_ep(s, s->bulk_in), buf, sizeof(buf));
        if (n < 0) {
            break;
        }
        if (n > 0) {
            qemu_chr_fe_write_all(&s->chr, buf, n);
        }
    }

    /* Drain serial state notifications */
    if (s->notify_in >= 0) {
        host_in(s, host_find_ep(s, s->notify_in), buf, sizeof(buf));
    }

    for (int i = 0; i < HOST_BULK_PACKETS && !fifo8_is_empty(&s->rx_fifo);
         i++) {
        int ep = host_find_ep(s, s->bulk_out);
        int len;

        if (ep < 0) {
            break;
        }
        len = MIN(fifo8_num_used(&s->rx_fifo),
                  MIN(rx_buffer_size(s, ep), sizeof(buf)));
        fifo8_peek_buf(&s->rx_fifo, buf, len);
        n = host_out(s, ep, buf, len);
        if (n <= 0) {
            break;
        }
        fifo8_drop(&s->rx_fifo, n);
        qemu_chr_fe_accept_input(&s->chr);
    }
}

static void host_frame(N32G4FRUsbdState *s)
{
    switch (s->host_state) {
    case HOST_RESET_WAIT:
        if (--s->host_wait <= 0) {
            host_bus_reset(s);
        }
        break;
    case HOST_ENABLE_WAIT:
        if (s->addr & ADDR_EFUC) {
            host_set_state(s, HOST_CONTROL);
            host_start_request(s);
        } else if (--s->host_timeout <= 0) {
            host_fail(s, "function not enabled after reset");
        }
        break;
    case HOST_CONTROL:
        for (int i = 0; i < 8 && s->host_state == HOST_CONTROL; i++) {
            if (!host_control_step(s)) {
                break;
            }
        }
        if (s->host_state == HOST_CONTROL && --s->host_timeout <= 0) {
            host_fail(s, "timeout");
        }
        break;
    case HOST_RUNNING:
        host_bulk(s);
        break;
    }
}

static void n32g4fr_usbd_frame(void *opaque)
{
    N32G4FRUsbdState *s = opaque;

    s->fn = (s->fn & ~FN_FNUM_MASK) | ((s->fn + 1) & FN_FNUM_MASK);
    s->sts |= STS_SOF;
    host_frame(s);
    n32g4fr_usbd_update_irq(s);

    if (s->host_state != HOST_DETACHED) {
        timer_mod(s->frame_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + FRAME_NS);
    }
}

/*
 * Attach when the firmware takes the controller out of reset/power down
 * with the DP pull-up enabled (that is what a host detects)
 */
static void n32g4fr_usbd_check_attach(N32G4FRUsbdState *s)
{
    bool powered = !(s->ctrl & (CTRL_FRST | CTRL_PD)) && s->dp_pullup;

    if (powered && s->host_state == HOST_DETACHED) {
        s->host_wait = HOST_ATTACH_FRAMES;
        s->host_retries = 0;
        host_set_state(s, HOST_RESET_WAIT);
        timer_mod(s->frame_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + FRAME_NS);
    } else if (!powered && s->host_state != HOST_DETACHED) {
        host_set_state(s, HOST_DETACHED);
        timer_del(s->frame_timer);
    }
}

static uint64_t n32g4fr_usbd_read(void *opaque, hwaddr addr,
                                  unsigned int size)
{
    N32G4FRUsbdState *s = N32G4FR_USBD(opaque);
    uint32_t value = 0;

    switch (addr) {
    case USB_EP0 ... USB_EP7:
        value = (addr & 3) ? 0 : s->epr[addr >> 2];
        break;
    case USB_CTRL:
        value = s->ctrl;
        break;
    case USB_STS:
        value = n32g4fr_usbd_sts(s);
        break;
    case USB_FN:
        /* Bus idle: full speed J state, D+ high */
        value = s->fn | FN_RXDP_STS;
        break;
    case USB_ADDR:
        value = s->addr;
        break;
    case USB_BUFTAB:
        value = s->buftab;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        break;
    }

    trace_n32g4fr_usbd_read(addr, value);
    return value;
}

static void n32g4fr_usbd_write_ep(N32G4FRUsbdState *s, int ep, uint32_t value)
{
    uint32_t old = s->epr[ep];

    if (extract32(value, EP_TYPE_SHIFT, 2) == EP_TYPE_ISO ||
        (extract32(value, EP_TYPE_SHIFT, 2) == EP_TYPE_BULK &&
         (value & EP_KIND))) {
        qemu_log_mask(LOG_UNIMP, "%s: isochronous and double buffered "
                      "endpoints are not implemented\n", __func__);
    }

    /*
     * EA/EP_TYPE/EP_KIND are read-write, DATTOG/STS toggle when written
     * with 1, CTRS are cleared by writing 0, SETUP is read-only.
     */
    s->epr[ep] = (value & EP_RW_MASK) |
                 ((old ^ value) & EP_TOGGLE_MASK) |
                 (old & value & EP_CTRS_MASK) |
                 (old & EP_SETUP);
}

static void n32g4fr_usbd_write(void *opaque, hwaddr addr,
                               uint64_t val64, unsigned int size)
{
    N32G4FRUsbdState *s = N32G4FR_USBD(opaque);
    uint32_t value = val64 & 0xFFFF;

    trace_n32g4fr_usbd_write(addr, value);

    switch (addr) {
    case USB_EP0 ... USB_EP7:
        if (!(addr & 3)) {
            n32g4fr_usbd_write_ep(s, addr >> 2, value);
        }
        break;
    case USB_CTRL:
        s->ctrl = value & CTRL_MASK;
        n32g4fr_usbd_check_attach(s);
        break;
    case USB_STS:
        /* Latched flags are cleared by writing 0 */
        s->sts &= value | ~STS_LATCHED;
        break;
    case USB_FN:
        break;
    case USB_ADDR:
        s->addr = value & 0xFF;
        break;
    case USB_BUFTAB:
        s->buftab = value & 0xFFF8;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        return;
    }

    n32g4fr_usbd_update_irq(s);
}

static const MemoryRegionOps n32g4fr_usbd_ops = {
    .read = n32g4fr_usbd_read,
    .write = n32g4fr_usbd_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/*
 * CPU view of the packet buffer memory: 16 bit word n at offset 4 * n,
 * the upper half of each 32 bit slot is not backed.
 */
static uint64_t n32g4fr_usbd_pma_read(void *opaque, hwaddr addr,
                                      unsigned int size)
{
    N32G4FRUsbdState *s = N32G4FR_USBD(opaque);
    uint32_t base = (addr >> 2) * 2;

    if (addr & 2) {
        return 0;
    }
    if (size == 1) {
        return s->pma[base + (addr & 1)];
    }
    return s->pma[base] | (s->pma[base + 1] << 8);
}

static void n32g4fr_usbd_pma_write(void *opaque, hwaddr addr,
                                   uint64_t value, unsigned int size)
{
    N32G4FRUsbdState *s = N32G4FR_USBD(opaque);
    uint32_t base = (addr >> 2) * 2;

    if (addr & 2) {
        return;
    }
    if (size == 1) {
        s->pma[base + (addr & 1)] = value;
        return;
    }
    s->pma[base] = value;
    s->pma[base + 1] = value >> 8;
}

static const MemoryRegionOps n32g4fr_usbd_pma_ops = {
    .read = n32g4fr_usbd_pma_read,
    .write = n32g4fr_usbd_pma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static int n32g4fr_usbd_can_receive(void *opaque)
{
    N32G4FRUsbdState *s = opaque;

    return s->host_state == HOST_RUNNING ? fifo8_num_free(&s->rx_fifo) : 0;
}

static void n32g4fr_usbd_receive(void *opaque, const uint8_t *buf, int size)
{
    N32G4FRUsbdState *s = opaque;

    fifo8_push_all(&s->rx_fifo, buf, MIN(size, fifo8_num_free(&s->rx_fifo)));
}

static void n32g4fr_usbd_reset(DeviceState *dev)
{
    N32G4FRUsbdState *s = N32G4FR_USBD(dev);

    memset(s->epr, 0, sizeof(s->epr));
    s->ctrl = CTRL_RESET;
    s->sts = 0;
    s->fn = 0;
    s->addr = 0;
    s->buftab = 0;
    s->host_state = HOST_DETACHED;
    s->enum_step = ENUM_GET_DEVICE;
    memset(s->dev_desc, 0, sizeof(s->dev_desc));
    s->bulk_in = s->bulk_out = s->notify_in = -1;
    fifo8_reset(&s->rx_fifo);
    timer_del(s->frame_timer);
    n32g4fr_usbd_update_irq(s);
}

static void n32g4fr_usbd_set_dp_pullup(void *opaque, int n, int level)
{
    N32G4FRUsbdState *s = N32G4FR_USBD(opaque);

    s->dp_pullup = level;
    n32g4fr_usbd_check_attach(s);
}

static void n32g4fr_usbd_init(Object *obj)
{
    N32G4FRUsbdState *s = N32G4FR_USBD(obj);

    memory_region_init_io(&s->mmio, obj, &n32g4fr_usbd_ops, s,
                          TYPE_N32G4FR_USBD, 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    memory_region_init_io(&s->pma_mmio, obj, &n32g4fr_usbd_pma_ops, s,
                          "n32g4fr.usbd-pma", 2 * N32G4FR_USBD_PMA_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->pma_mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq_lp);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq_hp);
    qdev_init_gpio_in_named(DEVICE(obj), n32g4fr_usbd_set_dp_pullup,
                            "dp-pullup", 1);
    s->dp_pullup = true;
}

static void n32g4fr_usbd_realize(DeviceState *dev, Error **errp)
{
    N32G4FRUsbdState *s = N32G4FR_USBD(dev);

    fifo8_create(&s->rx_fifo, RX_FIFO_SIZE);
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, n32g4fr_usbd_frame, s);
    qemu_chr_fe_set_handlers(&s->chr, n32g4fr_usbd_can_receive,
                             n32g4fr_usbd_receive, NULL, NULL, s, NULL, true);
}

static void n32g4fr_usbd_unrealize(DeviceState *dev)
{
    N32G4FRUsbdState *s = N32G4FR_USBD(dev);

    timer_free(s->frame_timer);
    fifo8_destroy(&s->rx_fifo);
}

static const VMStateDescription vmstate_n32g4fr_usbd = {
    .name = TYPE_N32G4FR_USBD,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(epr, N32G4FRUsbdState, N32G4FR_USBD_NUM_EPS),
        VMSTATE_UINT32(ctrl, N32G4FRUsbdState),
        VMSTATE_UINT32(sts, N32G4FRUsbdState),
        VMSTATE_UINT32(fn, N32G4FRUsbdState),
        VMSTATE_UINT32(addr, N32G4FRUsbdState),
        VMSTATE_UINT32(buftab, N32G4FRUsbdState),
        VMSTATE_UINT8_ARRAY(pma, N32G4FRUsbdState, N32G4FR_USBD_PMA_SIZE),
        VMSTATE_TIMER_PTR(frame_timer, N32G4FRUsbdState),
        VMSTATE_INT32(host_state, N32G4FRUsbdState),
        VMSTATE_INT32(host_wait, N32G4FRUsbdState),
        VMSTATE_INT32(host_timeout, N32G4FRUsbdState),
        VMSTATE_INT32(host_retries, N32G4FRUsbdState),
        VMSTATE_INT32(enum_step, N32G4FRUsbdState),
        VMSTATE_UINT8_ARRAY(setup, N32G4FRUsbdState, 8),
        VMSTATE_UINT8_ARRAY(ctrl_buf, N32G4FRUsbdState,
                            N32G4FR_USBD_CTRL_BUF),
        VMSTATE_INT32(ctrl_len, N32G4FRUsbdState),
        VMSTATE_INT32(ctrl_done, N32G4FRUsbdState),
        VMSTATE_INT32(ctrl_stage, N32G4FRUsbdState),
        VMSTATE_UINT8_ARRAY(dev_desc, N32G4FRUsbdState, 18),
        VMSTATE_UINT16(config_len, N32G4FRUsbdState),
        VMSTATE_UINT8(config_value, N32G4FRUsbdState),
        VMSTATE_UINT8(comm_iface, N32G4FRUsbdState),
        VMSTATE_INT8(bulk_in, N32G4FRUsbdState),
        VMSTATE_INT8(bulk_out, N32G4FRUsbdState),
        VMSTATE_INT8(notify_in, N32G4FRUsbdState),
        VMSTATE_FIFO8(rx_fifo, N32G4FRUsbdState),
        VMSTATE_BOOL(dp_pullup, N32G4FRUsbdState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property n32g4fr_usbd_properties[] = {
    DEFINE_PROP_CHR("chardev", N32G4FRUsbdState, chr),
};

static void n32g4fr_usbd_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = n32g4fr_usbd_realize;
    dc->unrealize = n32g4fr_usbd_unrealize;
    dc->vmsd = &vmstate_n32g4fr_usbd;
    device_class_set_legacy_reset(dc, n32g4fr_usbd_reset);
    device_class_set_props(dc, n32g4fr_usbd_properties);
}

static const TypeInfo n32g4fr_usbd_info = {
    .name          = TYPE_N32G4FR_USBD,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N32G4FRUsbdState),
    .instance_init = n32g4fr_usbd_init,
    .class_init    = n32g4fr_usbd_class_init,
};

static void n32g4fr_usbd_register_types(void)
{
    type_register_static(&n32g4fr_usbd_info);
}

type_init(n32g4fr_usbd_register_types)
