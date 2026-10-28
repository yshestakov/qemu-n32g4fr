/*
 * N32G4FR power control (PWR)
 *
 * Registers of N32G4FR User Manual V3.5.0 section "PWR registers". Low
 * power modes (sleep, stop, standby, VBAT) and the PVD are not modelled:
 * the status flags never set and PVDO reads 0 (VDD above the threshold).
 * PWR_CTRL2.IWDGRSTEN is forwarded to the IWDG, which only resets the chip
 * while it is set. CTRL4-CTRL7 are not in the user manual, only in
 * N32G4FR.svd (trim/analog controls); they are kept as plain registers.
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
#include "hw/misc/n32g4fr_pwr.h"

/* PWR_CTRL */
#define CTRL_CWKUP          BIT(2)
#define CTRL_CSBVBAT        BIT(3)
/* MSB, DBKP, PRS[2:0], PVDEN, PDS, LPS; CWKUP/CSBVBAT always read 0 */
#define CTRL_MASK           0x000003F3
/*
 * TODO confirm: the register description gives 0x0000_0700, the register
 * overview table resets bits 9:0 to 0. 0x700 would set the reserved bit 10
 * and leave the backup domain unprotected (DBKP) after reset, so use 0.
 */
#define CTRL_RESET          0x00000000

/* PWR_CTRLSTS */
#define CTRLSTS_WKUPF       BIT(0)
#define CTRLSTS_SBF         BIT(1)
#define CTRLSTS_VBATF       BIT(3)
#define CTRLSTS_WKUPEN      BIT(8)

/* PWR_CTRL2 */
#define CTRL2_IWDGRSTEN     BIT(10)
#define CTRL2_MASK          0x000007FF
#define CTRL2_RESET         0x000006E4

/* PWR_CTRL3: EXMODE (bit 0) plus undocumented trims in the reset value */
#define CTRL3_MASK          0x0007FFFF
#define CTRL3_RESET         0x00005B70

/* Writable bits of CTRL4-CTRL7 per N32G4FR.svd; reset values TODO confirm */
static const uint32_t ctrl4_7_mask[] = {
    0x000001FF, 0x000007FF, 0xFFFFFFFF, 0x00000001,
};

static void n32g4fr_pwr_update_iwdg(N32G4FRPwrState *s)
{
    qemu_set_irq(s->iwdg_reset_en,
                 !!(s->regs[N32G4FR_PWR_CTRL2 >> 2] & CTRL2_IWDGRSTEN));
}

static void n32g4fr_pwr_reset(DeviceState *dev)
{
    N32G4FRPwrState *s = N32G4FR_PWR(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[N32G4FR_PWR_CTRL >> 2] = CTRL_RESET;
    s->regs[N32G4FR_PWR_CTRL2 >> 2] = CTRL2_RESET;
    s->regs[N32G4FR_PWR_CTRL3 >> 2] = CTRL3_RESET;
    n32g4fr_pwr_update_iwdg(s);
}

static uint64_t n32g4fr_pwr_read(void *opaque, hwaddr addr, unsigned int size)
{
    N32G4FRPwrState *s = N32G4FR_PWR(opaque);
    uint32_t value = 0;

    if (addr > N32G4FR_PWR_CTRL7) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
    } else {
        value = s->regs[addr >> 2];
    }
    trace_n32g4fr_pwr_read(addr, value);
    return value;
}

static void n32g4fr_pwr_write(void *opaque, hwaddr addr,
                              uint64_t val64, unsigned int size)
{
    N32G4FRPwrState *s = N32G4FR_PWR(opaque);
    uint32_t value = val64;
    uint32_t *sts = &s->regs[N32G4FR_PWR_CTRLSTS >> 2];

    trace_n32g4fr_pwr_write(addr, value);

    switch (addr) {
    case N32G4FR_PWR_CTRL:
        /* Write-1 clear bits for the status flags */
        if (value & CTRL_CSBVBAT) {
            *sts &= ~(CTRLSTS_SBF | CTRLSTS_VBATF);
        }
        if (value & CTRL_CWKUP) {
            *sts &= ~CTRLSTS_WKUPF;
        }
        s->regs[addr >> 2] = value & CTRL_MASK;
        break;
    case N32G4FR_PWR_CTRLSTS:
        /* Only WKUPEN is writable, the flags are read-only */
        *sts = (*sts & ~CTRLSTS_WKUPEN) | (value & CTRLSTS_WKUPEN);
        break;
    case N32G4FR_PWR_CTRL2:
        s->regs[addr >> 2] = value & CTRL2_MASK;
        n32g4fr_pwr_update_iwdg(s);
        break;
    case N32G4FR_PWR_CTRL3:
        s->regs[addr >> 2] = value & CTRL3_MASK;
        break;
    case N32G4FR_PWR_CTRL4 ... N32G4FR_PWR_CTRL7:
        s->regs[addr >> 2] =
            value & ctrl4_7_mask[(addr - N32G4FR_PWR_CTRL4) >> 2];
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        break;
    }
}

static const MemoryRegionOps n32g4fr_pwr_ops = {
    .read = n32g4fr_pwr_read,
    .write = n32g4fr_pwr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void n32g4fr_pwr_init(Object *obj)
{
    N32G4FRPwrState *s = N32G4FR_PWR(obj);

    memory_region_init_io(&s->mmio, obj, &n32g4fr_pwr_ops, s,
                          TYPE_N32G4FR_PWR, N32G4FR_PWR_PERIPHERAL_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    qdev_init_gpio_out_named(DEVICE(obj), &s->iwdg_reset_en,
                             "iwdg-reset-en", 1);
}

static const VMStateDescription vmstate_n32g4fr_pwr = {
    .name = TYPE_N32G4FR_PWR,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, N32G4FRPwrState, N32G4FR_PWR_NREGS),
        VMSTATE_END_OF_LIST()
    }
};

static void n32g4fr_pwr_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_n32g4fr_pwr;
    device_class_set_legacy_reset(dc, n32g4fr_pwr_reset);
}

static const TypeInfo n32g4fr_pwr_info = {
    .name          = TYPE_N32G4FR_PWR,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N32G4FRPwrState),
    .instance_init = n32g4fr_pwr_init,
    .class_init    = n32g4fr_pwr_class_init,
};

static void n32g4fr_pwr_register_types(void)
{
    type_register_static(&n32g4fr_pwr_info);
}

type_init(n32g4fr_pwr_register_types)
