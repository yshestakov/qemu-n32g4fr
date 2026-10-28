/*
 * N32G4FR alternate function I/O (AFIO)
 *
 * Registers of N32G4FR User Manual V3.5.0 section "AFIO registers". The
 * remap and EXTI line selection registers are stored and read back, but
 * pin routing is not modelled (peripherals are not connected to GPIO pins),
 * so remaps have no effect; there is no EXTI either.
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
#include "trace.h"
#include "migration/vmstate.h"
#include "hw/misc/n32g4fr_afio.h"

/*
 * Writable bits per register, from the register overview table (Table
 * 6-47). N32G4FR.svd lists a few more bits in RMP_CFG (MII_RMII_SEL),
 * RMP_CFG3 (ETH_RMP) and RMP_CFG4 (COMP/ADC3/ADC4 remaps) that the manual
 * shows as reserved for this part; they are left out.
 */
static const uint32_t afio_mask[N32G4FR_AFIO_NREGS] = {
    [N32G4FR_AFIO_ECTRL >> 2]     = 0x000000FF,
    /* SW_JTAG_CFG[26:24] is write-only, see below */
    [N32G4FR_AFIO_RMP_CFG >> 2]   = 0x001FFFFF,
    [N32G4FR_AFIO_EXTI_CFG1 >> 2] = 0x0000FFFF,
    [N32G4FR_AFIO_EXTI_CFG2 >> 2] = 0x0000FFFF,
    [N32G4FR_AFIO_EXTI_CFG3 >> 2] = 0x0000FFFF,
    [N32G4FR_AFIO_EXTI_CFG4 >> 2] = 0x0000FFFF,
    [N32G4FR_AFIO_RMP_CFG3 >> 2]  = 0xCFFCFFF7,
    [N32G4FR_AFIO_RMP_CFG4 >> 2]  = 0x03F80000,
    [N32G4FR_AFIO_RMP_CFG5 >> 2]  = 0x00FFFFFF,
};

static bool n32g4fr_afio_valid(hwaddr addr)
{
    return addr <= N32G4FR_AFIO_RMP_CFG5 && afio_mask[addr >> 2];
}

static void n32g4fr_afio_reset(DeviceState *dev)
{
    N32G4FRAfioState *s = N32G4FR_AFIO(dev);

    /* Every register resets to 0 */
    memset(s->regs, 0, sizeof(s->regs));
}

static uint64_t n32g4fr_afio_read(void *opaque, hwaddr addr,
                                  unsigned int size)
{
    N32G4FRAfioState *s = N32G4FR_AFIO(opaque);
    uint32_t value = 0;

    if (!n32g4fr_afio_valid(addr)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
    } else {
        /* SW_JTAG_CFG reads as undefined, here 0 (it is not stored) */
        value = s->regs[addr >> 2];
    }
    trace_n32g4fr_afio_read(addr, value);
    return value;
}

static void n32g4fr_afio_write(void *opaque, hwaddr addr,
                               uint64_t val64, unsigned int size)
{
    N32G4FRAfioState *s = N32G4FR_AFIO(opaque);
    uint32_t value = val64;

    trace_n32g4fr_afio_write(addr, value);

    if (!n32g4fr_afio_valid(addr)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        return;
    }
    s->regs[addr >> 2] = value & afio_mask[addr >> 2];
}

static const MemoryRegionOps n32g4fr_afio_ops = {
    .read = n32g4fr_afio_read,
    .write = n32g4fr_afio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void n32g4fr_afio_init(Object *obj)
{
    N32G4FRAfioState *s = N32G4FR_AFIO(obj);

    memory_region_init_io(&s->mmio, obj, &n32g4fr_afio_ops, s,
                          TYPE_N32G4FR_AFIO, N32G4FR_AFIO_PERIPHERAL_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static const VMStateDescription vmstate_n32g4fr_afio = {
    .name = TYPE_N32G4FR_AFIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, N32G4FRAfioState, N32G4FR_AFIO_NREGS),
        VMSTATE_END_OF_LIST()
    }
};

static void n32g4fr_afio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_n32g4fr_afio;
    device_class_set_legacy_reset(dc, n32g4fr_afio_reset);
}

static const TypeInfo n32g4fr_afio_info = {
    .name          = TYPE_N32G4FR_AFIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N32G4FRAfioState),
    .instance_init = n32g4fr_afio_init,
    .class_init    = n32g4fr_afio_class_init,
};

static void n32g4fr_afio_register_types(void)
{
    type_register_static(&n32g4fr_afio_info);
}

type_init(n32g4fr_afio_register_types)
