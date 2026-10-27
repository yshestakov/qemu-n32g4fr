/*
 * N32G4FR SPI controller (master mode)
 *
 * Register layout per N32G4FR User Manual V3.5.0 (SPI chapter); it matches
 * the STM32F1 SPI. Transfers complete as soon as the data register is
 * written; the received word is then available in the data register.
 * CRC, I2S, slave and bidirectional modes are not modelled.
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
#include "migration/vmstate.h"
#include "hw/ssi/n32g4fr_spi.h"

#define SPI_CTRL1       0x00
#define SPI_CTRL2       0x04
#define SPI_STS         0x08
#define SPI_DAT         0x0C
#define SPI_CRCPOLY     0x10
#define SPI_CRCRDAT     0x14
#define SPI_CRCTDAT     0x18
#define SPI_I2SCFG      0x1C
#define SPI_I2SPREDIV   0x20

#define CTRL1_MSEL      BIT(2)
#define CTRL1_SPIEN     BIT(6)
#define CTRL1_DATFF     BIT(11)

#define CTRL2_RDMAEN    BIT(0)
#define CTRL2_TDMAEN    BIT(1)
#define CTRL2_ERRINTEN  BIT(5)
#define CTRL2_RNEINTEN  BIT(6)
#define CTRL2_TEINTEN   BIT(7)

#define STS_RNE         BIT(0)
#define STS_TE          BIT(1)
#define STS_OVER        BIT(6)

static void n32g4fr_spi_update(N32G4FRSpiState *s)
{
    bool enabled = s->ctrl1 & CTRL1_SPIEN;
    bool rx_pending = (s->sts & STS_RNE) && (s->ctrl2 & CTRL2_RDMAEN);
    bool irq = ((s->sts & STS_TE) && (s->ctrl2 & CTRL2_TEINTEN)) ||
               ((s->sts & STS_RNE) && (s->ctrl2 & CTRL2_RNEINTEN)) ||
               ((s->sts & STS_OVER) && (s->ctrl2 & CTRL2_ERRINTEN));

    qemu_set_irq(s->irq, enabled && irq);
    qemu_set_irq(s->dma_rx, rx_pending);
    /*
     * The transfer itself is instantaneous, so hold the TX request while
     * received data is still waiting for the RX DMA channel; on hardware
     * the frame time gives the RX channel that chance.
     */
    qemu_set_irq(s->dma_tx, enabled && (s->sts & STS_TE) &&
                 (s->ctrl2 & CTRL2_TDMAEN) && !rx_pending);
}

static void n32g4fr_spi_reset(DeviceState *dev)
{
    N32G4FRSpiState *s = N32G4FR_SPI(dev);

    s->ctrl1 = 0;
    s->ctrl2 = 0;
    s->sts = STS_TE;
    s->rx_dat = 0;
    s->crcpoly = 0x7;
    s->i2scfg = 0;
    s->i2sprediv = 0x2;
    n32g4fr_spi_update(s);
}

static void n32g4fr_spi_transfer(N32G4FRSpiState *s, uint32_t tx)
{
    uint32_t mask = (s->ctrl1 & CTRL1_DATFF) ? 0xFFFF : 0xFF;
    uint32_t rx;

    if (!(s->ctrl1 & CTRL1_SPIEN) || !(s->ctrl1 & CTRL1_MSEL)) {
        qemu_log_mask(LOG_GUEST_ERROR | LOG_UNIMP,
                      "%s: data written while not an enabled master\n",
                      __func__);
        return;
    }

    rx = ssi_transfer(s->ssi, tx & mask) & mask;
    trace_n32g4fr_spi_transfer(tx & mask, rx);

    if (s->sts & STS_RNE) {
        /* Overrun: the previous word was not read, the new one is lost */
        s->sts |= STS_OVER;
    } else {
        s->rx_dat = rx;
        s->sts |= STS_RNE;
    }
}

static uint64_t n32g4fr_spi_read(void *opaque, hwaddr addr, unsigned int size)
{
    N32G4FRSpiState *s = N32G4FR_SPI(opaque);
    uint32_t value = 0;

    switch (addr) {
    case SPI_CTRL1:
        value = s->ctrl1;
        break;
    case SPI_CTRL2:
        value = s->ctrl2;
        break;
    case SPI_STS:
        value = s->sts;
        /* Simplification: OVER is cleared by any STS read */
        s->sts &= ~STS_OVER;
        break;
    case SPI_DAT:
        value = s->rx_dat;
        s->sts &= ~STS_RNE;
        break;
    case SPI_CRCPOLY:
        value = s->crcpoly;
        break;
    case SPI_CRCRDAT:
    case SPI_CRCTDAT:
        qemu_log_mask(LOG_UNIMP, "%s: CRC is not implemented\n", __func__);
        break;
    case SPI_I2SCFG:
        value = s->i2scfg;
        break;
    case SPI_I2SPREDIV:
        value = s->i2sprediv;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        break;
    }

    trace_n32g4fr_spi_read(addr, value);
    n32g4fr_spi_update(s);
    return value;
}

static void n32g4fr_spi_write(void *opaque, hwaddr addr,
                              uint64_t val64, unsigned int size)
{
    N32G4FRSpiState *s = N32G4FR_SPI(opaque);
    uint32_t value = val64 & 0xFFFF;

    trace_n32g4fr_spi_write(addr, value);

    switch (addr) {
    case SPI_CTRL1:
        s->ctrl1 = value;
        break;
    case SPI_CTRL2:
        s->ctrl2 = value & 0xE7;
        break;
    case SPI_STS:
        /* Only CRCERR is writable (rc_w0); CRC is not modelled */
        break;
    case SPI_DAT:
        n32g4fr_spi_transfer(s, value);
        break;
    case SPI_CRCPOLY:
        s->crcpoly = value;
        break;
    case SPI_CRCRDAT:
    case SPI_CRCTDAT:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Read only register 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        break;
    case SPI_I2SCFG:
        if (value & BIT(11)) {
            qemu_log_mask(LOG_UNIMP, "%s: I2S is not implemented\n",
                          __func__);
        }
        s->i2scfg = value;
        break;
    case SPI_I2SPREDIV:
        s->i2sprediv = value;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        break;
    }

    n32g4fr_spi_update(s);
}

static const MemoryRegionOps n32g4fr_spi_ops = {
    .read = n32g4fr_spi_read,
    .write = n32g4fr_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static const VMStateDescription vmstate_n32g4fr_spi = {
    .name = TYPE_N32G4FR_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl1, N32G4FRSpiState),
        VMSTATE_UINT32(ctrl2, N32G4FRSpiState),
        VMSTATE_UINT32(sts, N32G4FRSpiState),
        VMSTATE_UINT32(rx_dat, N32G4FRSpiState),
        VMSTATE_UINT32(crcpoly, N32G4FRSpiState),
        VMSTATE_UINT32(i2scfg, N32G4FRSpiState),
        VMSTATE_UINT32(i2sprediv, N32G4FRSpiState),
        VMSTATE_END_OF_LIST()
    }
};

static void n32g4fr_spi_init(Object *obj)
{
    N32G4FRSpiState *s = N32G4FR_SPI(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &n32g4fr_spi_ops, s,
                          TYPE_N32G4FR_SPI, 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(dev, &s->dma_tx, N32G4FR_SPI_DMA_TX, 1);
    qdev_init_gpio_out_named(dev, &s->dma_rx, N32G4FR_SPI_DMA_RX, 1);

    s->ssi = ssi_create_bus(dev, "ssi");
}

static void n32g4fr_spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, n32g4fr_spi_reset);
    dc->vmsd = &vmstate_n32g4fr_spi;
}

static const TypeInfo n32g4fr_spi_info = {
    .name          = TYPE_N32G4FR_SPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N32G4FRSpiState),
    .instance_init = n32g4fr_spi_init,
    .class_init    = n32g4fr_spi_class_init,
};

static void n32g4fr_spi_register_types(void)
{
    type_register_static(&n32g4fr_spi_info);
}

type_init(n32g4fr_spi_register_types)
