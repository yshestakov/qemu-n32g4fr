/*
 * N32G4FR ADC (minimal)
 *
 * Enough of the ADC for firmware start-up and software triggered regular
 * conversions (N32G4FR User Manual V3.5.0 chapter 9):
 * - ADC_CTRL3.RDY follows ADC_CTRL2.ON, calibration completes at once;
 * - a software start (SWSTRRCH) converts the regular sequence, one channel
 *   per conversion period, in single or continuous mode, with ENDC/ENDCA
 *   flags, the interrupt and a DMA request per conversion;
 * - every conversion returns the "sample-value" property.
 * Injected channels, external triggers, the analog watchdog and dual mode
 * are not modelled.
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
#include "migration/vmstate.h"
#include "hw/adc/n32g4fr_adc.h"

#define ADC_STS         0x00
#define ADC_CTRL1       0x04
#define ADC_CTRL2       0x08
#define ADC_RSEQ1       0x2C
#define ADC_RSEQ2       0x30
#define ADC_RSEQ3       0x34
#define ADC_JSEQ        0x38
#define ADC_JDAT1       0x3C
#define ADC_JDAT4       0x48
#define ADC_DAT         0x4C
#define ADC_CTRL3       0x58

#define STS_ENDC        BIT(1)
#define STS_JENDC       BIT(2)
#define STS_STR         BIT(4)
#define STS_ENDCA       BIT(5)
#define STS_JENDCA      BIT(6)
#define STS_MASK        0x7F

#define CTRL1_ENDCIEN   BIT(5)
#define CTRL1_SCANMD    BIT(8)

#define CTRL2_ON        BIT(0)
#define CTRL2_CTU       BIT(1)
#define CTRL2_ENCAL     BIT(2)
#define CTRL2_ENDMA     BIT(8)
#define CTRL2_EXTRTRIG  BIT(20)
#define CTRL2_SWSTRJCH  BIT(21)
#define CTRL2_SWSTRRCH  BIT(22)

#define CTRL3_ENDCAIEN  BIT(8)
#define CTRL3_RDY       BIT(5)
#define CTRL3_PDRDY     BIT(6)
#define CTRL3_RESET     0x43

/* Duration of one conversion */
#define CONV_NS         1000

#define REG(s, off)     ((s)->regs[(off) >> 2])

static void n32g4fr_adc_update(N32G4FRAdcState *s)
{
    uint32_t sts = REG(s, ADC_STS);
    bool irq = ((sts & STS_ENDC) && (REG(s, ADC_CTRL1) & CTRL1_ENDCIEN)) ||
               ((sts & STS_ENDCA) && (REG(s, ADC_CTRL3) & CTRL3_ENDCAIEN));

    qemu_set_irq(s->irq, irq);
}

static int n32g4fr_adc_seq_len(N32G4FRAdcState *s)
{
    if (!(REG(s, ADC_CTRL1) & CTRL1_SCANMD)) {
        return 1;
    }
    return extract32(REG(s, ADC_RSEQ1), 20, 4) + 1;
}

static void n32g4fr_adc_schedule(N32G4FRAdcState *s)
{
    timer_mod(s->conv_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + CONV_NS);
}

static void n32g4fr_adc_convert(void *opaque)
{
    N32G4FRAdcState *s = opaque;

    if (s->seq_pos < 0 || !(REG(s, ADC_CTRL2) & CTRL2_ON)) {
        return;
    }

    REG(s, ADC_DAT) = s->sample_value & 0xFFFF;
    REG(s, ADC_STS) |= STS_ENDCA;
    trace_n32g4fr_adc_convert(s->seq_pos, REG(s, ADC_DAT));

    if (++s->seq_pos >= n32g4fr_adc_seq_len(s)) {
        /* End of the regular sequence */
        REG(s, ADC_STS) |= STS_ENDC;
        if (REG(s, ADC_CTRL2) & CTRL2_CTU) {
            s->seq_pos = 0;
        } else {
            s->seq_pos = -1;
            REG(s, ADC_STS) &= ~STS_STR;
        }
    }

    n32g4fr_adc_update(s);
    if (REG(s, ADC_CTRL2) & CTRL2_ENDMA) {
        /* Lowered when the DMA reads ADC_DAT */
        qemu_set_irq(s->dma, 1);
    }
    if (s->seq_pos >= 0) {
        n32g4fr_adc_schedule(s);
    }
}

static void n32g4fr_adc_reset(DeviceState *dev)
{
    N32G4FRAdcState *s = N32G4FR_ADC(dev);

    memset(s->regs, 0, sizeof(s->regs));
    REG(s, ADC_CTRL3) = CTRL3_RESET;
    s->seq_pos = -1;
    timer_del(s->conv_timer);
    n32g4fr_adc_update(s);
    qemu_set_irq(s->dma, 0);
}

static uint64_t n32g4fr_adc_read(void *opaque, hwaddr addr, unsigned int size)
{
    N32G4FRAdcState *s = N32G4FR_ADC(opaque);
    uint32_t value;

    if (addr >= N32G4FR_ADC_NREGS * 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        return 0;
    }

    value = s->regs[addr >> 2];
    if (addr == ADC_DAT) {
        REG(s, ADC_STS) &= ~STS_ENDC;
        n32g4fr_adc_update(s);
        qemu_set_irq(s->dma, 0);
    } else if (addr >= ADC_JDAT1 && addr <= ADC_JDAT4) {
        qemu_log_mask(LOG_UNIMP, "%s: injected channels not implemented\n",
                      __func__);
    }

    trace_n32g4fr_adc_read(addr, value);
    return value;
}

static void n32g4fr_adc_write(void *opaque, hwaddr addr,
                              uint64_t val64, unsigned int size)
{
    N32G4FRAdcState *s = N32G4FR_ADC(opaque);
    uint32_t value = val64;

    trace_n32g4fr_adc_write(addr, value);

    if (addr >= N32G4FR_ADC_NREGS * 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        return;
    }

    switch (addr) {
    case ADC_STS:
        /* Flags are cleared by writing 0 */
        REG(s, ADC_STS) &= value | ~STS_MASK;
        break;
    case ADC_CTRL2:
        if (value & CTRL2_SWSTRJCH) {
            qemu_log_mask(LOG_UNIMP, "%s: injected channels not "
                          "implemented\n", __func__);
        }
        if (value & CTRL2_EXTRTRIG) {
            qemu_log_mask(LOG_UNIMP, "%s: external triggers not "
                          "implemented\n", __func__);
        }
        /* Calibration completes at once; start bits are self-clearing */
        REG(s, ADC_CTRL2) = value & ~(CTRL2_ENCAL | CTRL2_SWSTRRCH |
                                      CTRL2_SWSTRJCH);
        if (value & CTRL2_ON) {
            REG(s, ADC_CTRL3) = (REG(s, ADC_CTRL3) | CTRL3_RDY) & ~CTRL3_PDRDY;
        } else {
            REG(s, ADC_CTRL3) = (REG(s, ADC_CTRL3) & ~CTRL3_RDY) | CTRL3_PDRDY;
            s->seq_pos = -1;
            timer_del(s->conv_timer);
        }
        if ((value & CTRL2_ON) && (value & CTRL2_SWSTRRCH) &&
            s->seq_pos < 0) {
            s->seq_pos = 0;
            REG(s, ADC_STS) |= STS_STR;
            n32g4fr_adc_schedule(s);
        }
        break;
    case ADC_DAT:
    case ADC_JDAT1 ... ADC_JDAT4:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read-only register 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        return;
    case ADC_CTRL3:
        /* RDY and PDRDY are read-only */
        REG(s, ADC_CTRL3) = (value & ~(CTRL3_RDY | CTRL3_PDRDY)) |
                            (REG(s, ADC_CTRL3) & (CTRL3_RDY | CTRL3_PDRDY));
        break;
    default:
        s->regs[addr >> 2] = value;
        break;
    }

    n32g4fr_adc_update(s);
}

static const MemoryRegionOps n32g4fr_adc_ops = {
    .read = n32g4fr_adc_read,
    .write = n32g4fr_adc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static const VMStateDescription vmstate_n32g4fr_adc = {
    .name = TYPE_N32G4FR_ADC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, N32G4FRAdcState, N32G4FR_ADC_NREGS),
        VMSTATE_INT32(seq_pos, N32G4FRAdcState),
        VMSTATE_TIMER_PTR(conv_timer, N32G4FRAdcState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property n32g4fr_adc_properties[] = {
    DEFINE_PROP_UINT32("sample-value", N32G4FRAdcState, sample_value, 0x800),
};

static void n32g4fr_adc_init(Object *obj)
{
    N32G4FRAdcState *s = N32G4FR_ADC(obj);

    memory_region_init_io(&s->mmio, obj, &n32g4fr_adc_ops, s,
                          TYPE_N32G4FR_ADC, 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), &s->dma, N32G4FR_ADC_DMA, 1);
}

static void n32g4fr_adc_realize(DeviceState *dev, Error **errp)
{
    N32G4FRAdcState *s = N32G4FR_ADC(dev);

    s->conv_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, n32g4fr_adc_convert, s);
}

static void n32g4fr_adc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = n32g4fr_adc_realize;
    dc->vmsd = &vmstate_n32g4fr_adc;
    device_class_set_legacy_reset(dc, n32g4fr_adc_reset);
    device_class_set_props(dc, n32g4fr_adc_properties);
}

static const TypeInfo n32g4fr_adc_info = {
    .name          = TYPE_N32G4FR_ADC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N32G4FRAdcState),
    .instance_init = n32g4fr_adc_init,
    .class_init    = n32g4fr_adc_class_init,
};

static void n32g4fr_adc_register_types(void)
{
    type_register_static(&n32g4fr_adc_info);
}

type_init(n32g4fr_adc_register_types)
