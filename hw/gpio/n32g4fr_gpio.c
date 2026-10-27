/*
 * N32G4FR GPIO port
 *
 * STM32F1-style port (N32G4FR User Manual V3.5.0, GPIO chapter): two
 * configuration registers with a 2 bit mode and 2 bit configuration per
 * pin, input/output data, bit set/clear and lock registers.
 *
 * Pins configured as outputs drive their POD bit on the matching GPIO
 * output line, so e.g. SPI chip selects can be wired to them. Input pins
 * read the level driven on the matching GPIO input line; pull-up/pull-down
 * inputs follow POD while nothing drives them. Alternate function outputs
 * are reported as outputs of their POD bit; the peripheral side is not
 * modelled. Configuration locking is not enforced.
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
#include "hw/gpio/n32g4fr_gpio.h"

#define GPIO_PL_CFG     0x00
#define GPIO_PH_CFG     0x04
#define GPIO_PID        0x08
#define GPIO_POD        0x0C
#define GPIO_PBSC       0x10
#define GPIO_PBC        0x14
#define GPIO_PLOCK      0x18

#define PIN_MASK        0xFFFF

/* 4 bit pin configuration field: PMODE[1:0] in bits 1:0, PCFG[1:0] in 3:2 */
static uint32_t n32g4fr_gpio_pin_cfg(N32G4FRGpioState *s, int pin)
{
    uint32_t reg = pin < 8 ? s->pl_cfg : s->ph_cfg;

    return extract32(reg, (pin % 8) * 4, 4);
}

static uint32_t n32g4fr_gpio_output_mask(N32G4FRGpioState *s)
{
    uint32_t mask = 0;

    for (int pin = 0; pin < N32G4FR_GPIO_NUM_PINS; pin++) {
        if (n32g4fr_gpio_pin_cfg(s, pin) & 0x3) {
            mask |= BIT(pin);
        }
    }
    return mask;
}

static uint32_t n32g4fr_gpio_pid(N32G4FRGpioState *s)
{
    uint32_t outputs = n32g4fr_gpio_output_mask(s);
    uint32_t pulled = 0;

    for (int pin = 0; pin < N32G4FR_GPIO_NUM_PINS; pin++) {
        if (n32g4fr_gpio_pin_cfg(s, pin) == 0x8) {
            pulled |= BIT(pin);
        }
    }
    uint32_t undriven = ~outputs & ~s->ext_driven;

    /* Undriven inputs read their pull (POD) if pulled, else 0 */
    return (s->pod & outputs) |
           (s->ext_level & ~outputs & s->ext_driven) |
           (s->pod & pulled & undriven);
}

static void n32g4fr_gpio_update(N32G4FRGpioState *s)
{
    uint32_t outputs = n32g4fr_gpio_output_mask(s);
    uint32_t level = s->pod & outputs;
    /* Pins that just became outputs drive their level even if unchanged */
    uint32_t changed = ((level ^ s->out_level) & outputs) |
                       (outputs & ~s->out_mask);

    for (int pin = 0; pin < N32G4FR_GPIO_NUM_PINS; pin++) {
        if (changed & BIT(pin)) {
            trace_n32g4fr_gpio_pin(s->name, pin,
                                   extract32(level, pin, 1));
            qemu_set_irq(s->out[pin], extract32(level, pin, 1));
        }
    }
    s->out_level = (s->out_level & ~outputs) | level;
    s->out_mask = outputs;
}

static void n32g4fr_gpio_set_input(void *opaque, int pin, int level)
{
    N32G4FRGpioState *s = N32G4FR_GPIO(opaque);

    s->ext_level = deposit32(s->ext_level, pin, 1, !!level);
    s->ext_driven |= BIT(pin);
}

static void n32g4fr_gpio_reset(DeviceState *dev)
{
    N32G4FRGpioState *s = N32G4FR_GPIO(dev);

    s->pl_cfg = s->pl_cfg_reset;
    s->ph_cfg = s->ph_cfg_reset;
    s->pod = s->pod_reset;
    s->plock = 0;
    s->out_mask = 0;
    n32g4fr_gpio_update(s);
}

static uint64_t n32g4fr_gpio_read(void *opaque, hwaddr addr,
                                  unsigned int size)
{
    N32G4FRGpioState *s = N32G4FR_GPIO(opaque);
    uint32_t value = 0;

    switch (addr) {
    case GPIO_PL_CFG:
        value = s->pl_cfg;
        break;
    case GPIO_PH_CFG:
        value = s->ph_cfg;
        break;
    case GPIO_PID:
        value = n32g4fr_gpio_pid(s);
        break;
    case GPIO_POD:
        value = s->pod;
        break;
    case GPIO_PBSC:
    case GPIO_PBC:
        /* Write-only */
        break;
    case GPIO_PLOCK:
        value = s->plock;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        break;
    }

    trace_n32g4fr_gpio_read(s->name, addr, value);
    return value;
}

static void n32g4fr_gpio_write(void *opaque, hwaddr addr,
                               uint64_t val64, unsigned int size)
{
    N32G4FRGpioState *s = N32G4FR_GPIO(opaque);
    uint32_t value = val64;

    trace_n32g4fr_gpio_write(s->name, addr, value);

    switch (addr) {
    case GPIO_PL_CFG:
        s->pl_cfg = value;
        break;
    case GPIO_PH_CFG:
        s->ph_cfg = value;
        break;
    case GPIO_PID:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: GPIOx_PID is read-only\n",
                      __func__);
        return;
    case GPIO_POD:
        s->pod = value & PIN_MASK;
        break;
    case GPIO_PBSC:
        /* Set wins over clear when both bits are written */
        s->pod &= ~(value >> 16);
        s->pod |= value & PIN_MASK;
        break;
    case GPIO_PBC:
        s->pod &= ~(value & PIN_MASK);
        break;
    case GPIO_PLOCK:
        s->plock = value & 0x1FFFF;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        return;
    }

    n32g4fr_gpio_update(s);
}

static const MemoryRegionOps n32g4fr_gpio_ops = {
    .read = n32g4fr_gpio_read,
    .write = n32g4fr_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static const VMStateDescription vmstate_n32g4fr_gpio = {
    .name = TYPE_N32G4FR_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(pl_cfg, N32G4FRGpioState),
        VMSTATE_UINT32(ph_cfg, N32G4FRGpioState),
        VMSTATE_UINT32(pod, N32G4FRGpioState),
        VMSTATE_UINT32(plock, N32G4FRGpioState),
        VMSTATE_UINT32(ext_level, N32G4FRGpioState),
        VMSTATE_UINT32(ext_driven, N32G4FRGpioState),
        VMSTATE_UINT32(out_level, N32G4FRGpioState),
        VMSTATE_UINT32(out_mask, N32G4FRGpioState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property n32g4fr_gpio_properties[] = {
    DEFINE_PROP_UINT32("pl-cfg-reset", N32G4FRGpioState, pl_cfg_reset, 0),
    DEFINE_PROP_UINT32("ph-cfg-reset", N32G4FRGpioState, ph_cfg_reset, 0),
    DEFINE_PROP_UINT32("pod-reset", N32G4FRGpioState, pod_reset, 0),
};

static void n32g4fr_gpio_init(Object *obj)
{
    N32G4FRGpioState *s = N32G4FR_GPIO(obj);

    memory_region_init_io(&s->mmio, obj, &n32g4fr_gpio_ops, s,
                          TYPE_N32G4FR_GPIO, 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    qdev_init_gpio_out(DEVICE(obj), s->out, N32G4FR_GPIO_NUM_PINS);
    qdev_init_gpio_in(DEVICE(obj), n32g4fr_gpio_set_input,
                      N32G4FR_GPIO_NUM_PINS);
}

static void n32g4fr_gpio_realize(DeviceState *dev, Error **errp)
{
    N32G4FRGpioState *s = N32G4FR_GPIO(dev);

    s->name = g_strdup(object_get_canonical_path_component(OBJECT(dev)));
}

static void n32g4fr_gpio_finalize(Object *obj)
{
    N32G4FRGpioState *s = N32G4FR_GPIO(obj);

    g_free(s->name);
}

static void n32g4fr_gpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = n32g4fr_gpio_realize;
    dc->vmsd = &vmstate_n32g4fr_gpio;
    device_class_set_legacy_reset(dc, n32g4fr_gpio_reset);
    device_class_set_props(dc, n32g4fr_gpio_properties);
}

static const TypeInfo n32g4fr_gpio_info = {
    .name          = TYPE_N32G4FR_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N32G4FRGpioState),
    .instance_init = n32g4fr_gpio_init,
    .instance_finalize = n32g4fr_gpio_finalize,
    .class_init    = n32g4fr_gpio_class_init,
};

static void n32g4fr_gpio_register_types(void)
{
    type_register_static(&n32g4fr_gpio_info);
}

type_init(n32g4fr_gpio_register_types)
