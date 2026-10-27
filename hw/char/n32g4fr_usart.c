/*
 * N32G4FR USART / UART
 *
 * Based on the STM32F2XX USART model,
 * Copyright (c) 2014 Alistair Francis <alistair@alistair23.me>
 *
 * Register layout per N32G4FR User Manual V3.5.0 (USART chapter), which
 * matches the STM32F1 USART. Adds over the STM32F2XX model: DMA requests
 * (USART_CTRL3.DMATXEN/DMARXEN), the IDLE line flag that DMA based
 * receivers use to detect the end of a burst, and the STS-then-DAT read
 * sequence that clears IDLEF/OREF.
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
#include "qemu/module.h"
#include "trace.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "hw/char/n32g4fr_usart.h"

#define USART_STS       0x00
#define USART_DAT       0x04
#define USART_BRCF      0x08
#define USART_CTRL1     0x0C
#define USART_CTRL2     0x10
#define USART_CTRL3     0x14
#define USART_GTP       0x18

#define STS_OREF        BIT(3)
#define STS_IDLEF       BIT(4)
#define STS_RXDNE       BIT(5)
#define STS_TXC         BIT(6)
#define STS_TXDE        BIT(7)
#define STS_LINBDF      BIT(8)
#define STS_CTSF        BIT(9)
/* Flags software clears by writing 0 */
#define STS_RC_W0       (STS_RXDNE | STS_TXC | STS_LINBDF | STS_CTSF)
#define STS_RESET       (STS_TXDE | STS_TXC)

#define CTRL1_RXEN      BIT(2)
#define CTRL1_TXEN      BIT(3)
#define CTRL1_IDLEIEN   BIT(4)
#define CTRL1_RXDNEIEN  BIT(5)
#define CTRL1_TXCIEN    BIT(6)
#define CTRL1_TXDEIEN   BIT(7)
#define CTRL1_UEN       BIT(13)

#define CTRL3_DMARXEN   BIT(6)
#define CTRL3_DMATXEN   BIT(7)

/* Frame time used for IDLE detection when BRCF is not programmed */
#define DEFAULT_FRAME_NS (100 * SCALE_US)

static int n32g4fr_usart_can_receive(void *opaque)
{
    N32G4FRUsartState *s = opaque;

    return !(s->sts & STS_RXDNE);
}

static void n32g4fr_usart_update(N32G4FRUsartState *s)
{
    bool enabled = s->ctrl1 & CTRL1_UEN;
    /* TXDE/TXC/RXDNE/IDLEF line up with their CTRL1 interrupt enables */
    uint32_t mask = s->sts & s->ctrl1 &
                    (STS_TXDE | STS_TXC | STS_RXDNE | STS_IDLEF);

    if ((s->sts & STS_OREF) && (s->ctrl1 & CTRL1_RXDNEIEN)) {
        mask |= STS_OREF;
    }
    qemu_set_irq(s->irq, enabled && mask);
    qemu_set_irq(s->dma_rx, enabled && (s->sts & STS_RXDNE) &&
                 (s->ctrl3 & CTRL3_DMARXEN));
    qemu_set_irq(s->dma_tx, enabled && (s->ctrl1 & CTRL1_TXEN) &&
                 (s->sts & STS_TXDE) && (s->ctrl3 & CTRL3_DMATXEN));
}

/* Duration of one 10 bit frame at the programmed baud rate */
static int64_t n32g4fr_usart_frame_ns(N32G4FRUsartState *s)
{
    /* BRCF = pclk / baud (mantissa << 4 | fraction) */
    if (!s->brcf || !s->pclk_frequency) {
        return DEFAULT_FRAME_NS;
    }
    return muldiv64(10 * NANOSECONDS_PER_SECOND, s->brcf, s->pclk_frequency);
}

static void n32g4fr_usart_idle(void *opaque)
{
    N32G4FRUsartState *s = opaque;

    if (s->idle_armed) {
        s->idle_armed = false;
        s->sts |= STS_IDLEF;
        trace_n32g4fr_usart_idle(DEVICE(s)->id);
        n32g4fr_usart_update(s);
    }
}

static void n32g4fr_usart_receive(void *opaque, const uint8_t *buf, int size)
{
    N32G4FRUsartState *s = opaque;
    DeviceState *d = DEVICE(s);

    if (!(s->ctrl1 & CTRL1_UEN && s->ctrl1 & CTRL1_RXEN)) {
        /* USART not enabled - drop the chars */
        trace_n32g4fr_usart_drop(d->id);
        return;
    }

    s->dat = *buf;
    s->sts |= STS_RXDNE;
    s->idle_armed = true;
    timer_mod(s->idle_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              n32g4fr_usart_frame_ns(s));

    n32g4fr_usart_update(s);

    trace_n32g4fr_usart_receive(d->id, *buf);
}

static void n32g4fr_usart_reset(DeviceState *dev)
{
    N32G4FRUsartState *s = N32G4FR_USART(dev);

    s->sts = STS_RESET;
    s->dat = 0;
    s->brcf = 0;
    s->ctrl1 = 0;
    s->ctrl2 = 0;
    s->ctrl3 = 0;
    s->gtp = 0;
    s->sts_read = false;
    s->idle_armed = false;
    timer_del(s->idle_timer);

    n32g4fr_usart_update(s);
}

static uint64_t n32g4fr_usart_read(void *opaque, hwaddr addr,
                                   unsigned int size)
{
    N32G4FRUsartState *s = opaque;
    DeviceState *d = DEVICE(s);
    uint64_t retvalue = 0;

    switch (addr) {
    case USART_STS:
        retvalue = s->sts;
        s->sts_read = true;
        qemu_chr_fe_accept_input(&s->chr);
        break;
    case USART_DAT:
        retvalue = s->dat & 0x1FF;
        s->sts &= ~STS_RXDNE;
        if (s->sts_read) {
            s->sts &= ~(STS_IDLEF | STS_OREF);
            s->sts_read = false;
        }
        n32g4fr_usart_update(s);
        qemu_chr_fe_accept_input(&s->chr);
        break;
    case USART_BRCF:
        retvalue = s->brcf;
        break;
    case USART_CTRL1:
        retvalue = s->ctrl1;
        break;
    case USART_CTRL2:
        retvalue = s->ctrl2;
        break;
    case USART_CTRL3:
        retvalue = s->ctrl3;
        break;
    case USART_GTP:
        retvalue = s->gtp;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, addr);
        return 0;
    }

    trace_n32g4fr_usart_read(d->id, size, addr, retvalue);

    return retvalue;
}

static void n32g4fr_usart_write(void *opaque, hwaddr addr,
                                uint64_t val64, unsigned int size)
{
    N32G4FRUsartState *s = opaque;
    DeviceState *d = DEVICE(s);
    uint32_t value = val64;
    unsigned char ch;

    trace_n32g4fr_usart_write(d->id, size, addr, val64);

    switch (addr) {
    case USART_STS:
        s->sts &= value | ~STS_RC_W0;
        break;
    case USART_DAT:
        if ((s->ctrl1 & CTRL1_UEN) && (s->ctrl1 & CTRL1_TXEN)) {
            ch = value;
            /*
             * XXX this blocks entire thread. Rewrite to use
             * qemu_chr_fe_write and background I/O callbacks
             */
            qemu_chr_fe_write_all(&s->chr, &ch, 1);
        }
        /*
         * I/O is synchronous: TXDE stays set and TXC is set again after
         * every write, since software may have cleared it.
         */
        s->sts |= STS_TXC;
        break;
    case USART_BRCF:
        s->brcf = value & 0xFFFF;
        break;
    case USART_CTRL1:
        s->ctrl1 = value & 0x3FFF;
        break;
    case USART_CTRL2:
        s->ctrl2 = value & 0x7F7F;
        break;
    case USART_CTRL3:
        s->ctrl3 = value & 0x7FF;
        break;
    case USART_GTP:
        s->gtp = value & 0xFFFF;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, addr);
        return;
    }

    n32g4fr_usart_update(s);
}

static const MemoryRegionOps n32g4fr_usart_ops = {
    .read = n32g4fr_usart_read,
    .write = n32g4fr_usart_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static const VMStateDescription vmstate_n32g4fr_usart = {
    .name = TYPE_N32G4FR_USART,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(sts, N32G4FRUsartState),
        VMSTATE_UINT32(dat, N32G4FRUsartState),
        VMSTATE_UINT32(brcf, N32G4FRUsartState),
        VMSTATE_UINT32(ctrl1, N32G4FRUsartState),
        VMSTATE_UINT32(ctrl2, N32G4FRUsartState),
        VMSTATE_UINT32(ctrl3, N32G4FRUsartState),
        VMSTATE_UINT32(gtp, N32G4FRUsartState),
        VMSTATE_BOOL(sts_read, N32G4FRUsartState),
        VMSTATE_BOOL(idle_armed, N32G4FRUsartState),
        VMSTATE_TIMER_PTR(idle_timer, N32G4FRUsartState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property n32g4fr_usart_properties[] = {
    DEFINE_PROP_CHR("chardev", N32G4FRUsartState, chr),
    DEFINE_PROP_UINT32("pclk-frequency", N32G4FRUsartState, pclk_frequency,
                       36000000),
};

static void n32g4fr_usart_init(Object *obj)
{
    N32G4FRUsartState *s = N32G4FR_USART(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), &s->dma_tx, N32G4FR_USART_DMA_TX, 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->dma_rx, N32G4FR_USART_DMA_RX, 1);

    memory_region_init_io(&s->mmio, obj, &n32g4fr_usart_ops, s,
                          TYPE_N32G4FR_USART, 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static void n32g4fr_usart_realize(DeviceState *dev, Error **errp)
{
    N32G4FRUsartState *s = N32G4FR_USART(dev);

    s->idle_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, n32g4fr_usart_idle, s);
    qemu_chr_fe_set_handlers(&s->chr, n32g4fr_usart_can_receive,
                             n32g4fr_usart_receive, NULL, NULL,
                             s, NULL, true);
}

static void n32g4fr_usart_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, n32g4fr_usart_reset);
    device_class_set_props(dc, n32g4fr_usart_properties);
    dc->realize = n32g4fr_usart_realize;
    dc->vmsd = &vmstate_n32g4fr_usart;
}

static const TypeInfo n32g4fr_usart_info = {
    .name          = TYPE_N32G4FR_USART,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N32G4FRUsartState),
    .instance_init = n32g4fr_usart_init,
    .class_init    = n32g4fr_usart_class_init,
};

static void n32g4fr_usart_register_types(void)
{
    type_register_static(&n32g4fr_usart_info);
}

type_init(n32g4fr_usart_register_types)
