/*
 * N32G4FR undocumented system control block at 0x40001800
 *
 * The User Manual V3.5.0 memory map marks 0x40001800-0x40001BFF as
 * reserved, but its USB chapter says the pull-up resistor on the USB DP
 * line is enabled by bit 28 of the register at 0x40001820, and that the
 * other bits of that register must not be modified. Bit 28 drives the
 * "usb-dp-pullup" GPIO (the USB device's attach signal). The rest of the
 * block is unknown: accesses are stored and read back, and logged with
 * -d unimp.
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
#include "hw/core/irq.h"
#include "trace.h"
#include "migration/vmstate.h"
#include "hw/misc/n32g4fr_sysctl.h"

#define USB_PU_DP_PULLUP    BIT(28)

static void n32g4fr_sysctl_update(N32G4FRSysctlState *s)
{
    qemu_set_irq(s->usb_dp_pullup,
                 !!(s->regs[N32G4FR_SYSCTL_USB_PU >> 2] & USB_PU_DP_PULLUP));
}

static void n32g4fr_sysctl_reset(DeviceState *dev)
{
    N32G4FRSysctlState *s = N32G4FR_SYSCTL(dev);

    /* TODO confirm: reset values are undocumented; DP pull-up is off */
    memset(s->regs, 0, sizeof(s->regs));
    n32g4fr_sysctl_update(s);
}

static uint64_t n32g4fr_sysctl_read(void *opaque, hwaddr addr,
                                    unsigned int size)
{
    N32G4FRSysctlState *s = N32G4FR_SYSCTL(opaque);
    uint32_t value = s->regs[addr >> 2];

    if (addr != N32G4FR_SYSCTL_USB_PU) {
        qemu_log_mask(LOG_UNIMP, "%s: undocumented register 0x%"HWADDR_PRIx
                      "\n", __func__, addr);
    }
    trace_n32g4fr_sysctl_read(addr, value);
    return value;
}

static void n32g4fr_sysctl_write(void *opaque, hwaddr addr,
                                 uint64_t val64, unsigned int size)
{
    N32G4FRSysctlState *s = N32G4FR_SYSCTL(opaque);
    uint32_t value = val64;
    uint32_t *reg = &s->regs[addr >> 2];

    trace_n32g4fr_sysctl_write(addr, value);

    if (addr != N32G4FR_SYSCTL_USB_PU) {
        qemu_log_mask(LOG_UNIMP, "%s: undocumented register 0x%"HWADDR_PRIx
                      " = 0x%08x\n", __func__, addr, value);
    } else if ((value ^ *reg) & ~USB_PU_DP_PULLUP) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: 0x%"HWADDR_PRIx": only bit 28 "
                      "may be modified (0x%08x -> 0x%08x)\n", __func__, addr,
                      *reg, value);
    }
    *reg = value;
    n32g4fr_sysctl_update(s);
}

static const MemoryRegionOps n32g4fr_sysctl_ops = {
    .read = n32g4fr_sysctl_read,
    .write = n32g4fr_sysctl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void n32g4fr_sysctl_init(Object *obj)
{
    N32G4FRSysctlState *s = N32G4FR_SYSCTL(obj);

    memory_region_init_io(&s->mmio, obj, &n32g4fr_sysctl_ops, s,
                          TYPE_N32G4FR_SYSCTL, N32G4FR_SYSCTL_PERIPHERAL_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    qdev_init_gpio_out_named(DEVICE(obj), &s->usb_dp_pullup,
                             "usb-dp-pullup", 1);
}

static const VMStateDescription vmstate_n32g4fr_sysctl = {
    .name = TYPE_N32G4FR_SYSCTL,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, N32G4FRSysctlState, N32G4FR_SYSCTL_NREGS),
        VMSTATE_END_OF_LIST()
    }
};

static void n32g4fr_sysctl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_n32g4fr_sysctl;
    device_class_set_legacy_reset(dc, n32g4fr_sysctl_reset);
}

static const TypeInfo n32g4fr_sysctl_info = {
    .name          = TYPE_N32G4FR_SYSCTL,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N32G4FRSysctlState),
    .instance_init = n32g4fr_sysctl_init,
    .class_init    = n32g4fr_sysctl_class_init,
};

static void n32g4fr_sysctl_register_types(void)
{
    type_register_static(&n32g4fr_sysctl_info);
}

type_init(n32g4fr_sysctl_register_types)
