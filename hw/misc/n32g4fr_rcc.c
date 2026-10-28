/*
 * N32G4FR RCC (clock ready/switch status is faked, nothing is clocked)
 *
 * Oscillators and the PLL report ready as soon as they are enabled, and the
 * system clock switch status follows the requested source immediately, so
 * that firmware polling loops in SystemInit terminate. Clock frequencies are
 * not modelled: the SoC sysclk stays at whatever the board set.
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
#include "migration/vmstate.h"
#include "hw/misc/n32g4fr_rcc.h"

/* Bit positions from N32G4FR User Manual V3.5.0 section 5.3 */

/* RCC_CTRL */
#define CTRL_HSIEN          BIT(0)
#define CTRL_HSIRDF         BIT(1)
#define CTRL_HSICAL_MASK    0x0000FF00
#define CTRL_HSEEN          BIT(16)
#define CTRL_HSERDF         BIT(17)
#define CTRL_PLLEN          BIT(24)
#define CTRL_PLLRDF         BIT(25)
#define CTRL_RO_MASK        (CTRL_HSIRDF | CTRL_HSICAL_MASK | \
                             CTRL_HSERDF | CTRL_PLLRDF)

/* RCC_CFG */
#define CFG_SCLKSW_SHIFT    0
#define CFG_SCLKSTS_SHIFT   2
#define CFG_SCLK_LEN        2

/* RCC_CLKINT: bits 12:8 are interrupt enables, flags and clears are not kept */
#define CLKINT_IEN_MASK     0x00001F00

/* RCC_BDCTRL */
#define BDCTRL_LSEEN        BIT(0)
#define BDCTRL_LSERD        BIT(1)

/* RCC_CTRLSTS */
#define CTRLSTS_LSIEN       BIT(0)
#define CTRLSTS_LSIRD       BIT(1)
#define CTRLSTS_RMRSTF      BIT(24)
#define CTRLSTS_IWDGRSTF    BIT(29)
#define CTRLSTS_FLAGS_MASK  0xFEB80000  /* bits 31:25, 23, 21:19 */

/* Reset values */
#define CTRL_RESET          0x00000083
#define CFG_RESET           0x20000000
#define AHBPCLKEN_RESET     0x00000014
#define CTRLSTS_RESET       0x0C000003  /* LSI on and ready, POR + pin reset */
#define CFG2_RESET          0x00003800
#define CFG3_RESET          0x00003800

static void n32g4fr_rcc_reset(DeviceState *dev)
{
    N32G4FRRccState *s = N32G4FR_RCC(dev);

    for (int i = 0; i < N32G4FR_RCC_NREGS; i++) {
        s->regs[i] = 0;
    }
    s->regs[N32G4FR_RCC_CTRL >> 2] = CTRL_RESET;
    s->regs[N32G4FR_RCC_CFG >> 2] = CFG_RESET;
    s->regs[N32G4FR_RCC_AHBPCLKEN >> 2] = AHBPCLKEN_RESET;
    s->regs[N32G4FR_RCC_CTRLSTS >> 2] = CTRLSTS_RESET | s->pending_rst_flags;
    s->pending_rst_flags = 0;
    s->regs[N32G4FR_RCC_CFG2 >> 2] = CFG2_RESET;
    s->regs[N32G4FR_RCC_CFG3 >> 2] = CFG3_RESET;
}

/* The IWDG resets the chip: report it in CTRLSTS after that reset */
static void n32g4fr_rcc_iwdg_reset(void *opaque, int n, int level)
{
    N32G4FRRccState *s = N32G4FR_RCC(opaque);

    if (level) {
        s->pending_rst_flags |= CTRLSTS_IWDGRSTF;
    }
}

static uint64_t n32g4fr_rcc_read(void *opaque, hwaddr addr, unsigned int size)
{
    N32G4FRRccState *s = N32G4FR_RCC(opaque);
    uint32_t value = 0;

    if (addr > N32G4FR_RCC_CFG3) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
    } else {
        value = s->regs[addr >> 2];
    }
    trace_n32g4fr_rcc_read(addr, value);
    return value;
}

static void n32g4fr_rcc_write(void *opaque, hwaddr addr,
                              uint64_t val64, unsigned int size)
{
    N32G4FRRccState *s = N32G4FR_RCC(opaque);
    uint32_t value = val64;
    uint32_t old;

    trace_n32g4fr_rcc_write(addr, value);

    if (addr > N32G4FR_RCC_CFG3) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        return;
    }

    old = s->regs[addr >> 2];

    switch (addr) {
    case N32G4FR_RCC_CTRL:
        /* Every enabled oscillator/PLL is instantly ready */
        value = (value & ~CTRL_RO_MASK) | (old & CTRL_HSICAL_MASK);
        if (value & CTRL_HSIEN) {
            value |= CTRL_HSIRDF;
        }
        if (value & CTRL_HSEEN) {
            value |= CTRL_HSERDF;
        }
        if (value & CTRL_PLLEN) {
            value |= CTRL_PLLRDF;
        }
        break;
    case N32G4FR_RCC_CFG:
        /* The system clock switch completes instantly */
        value = deposit32(value, CFG_SCLKSTS_SHIFT, CFG_SCLK_LEN,
                          extract32(value, CFG_SCLKSW_SHIFT, CFG_SCLK_LEN));
        break;
    case N32G4FR_RCC_CLKINT:
        value &= CLKINT_IEN_MASK;
        break;
    case N32G4FR_RCC_BDCTRL:
        value &= ~BDCTRL_LSERD;
        if (value & BDCTRL_LSEEN) {
            value |= BDCTRL_LSERD;
        }
        break;
    case N32G4FR_RCC_CTRLSTS:
        /* Reset flags are read-only; writing RMRSTF clears them */
        value = (value & ~(CTRLSTS_FLAGS_MASK | CTRLSTS_LSIRD |
                           CTRLSTS_RMRSTF)) |
                (old & CTRLSTS_FLAGS_MASK);
        if (val64 & CTRLSTS_RMRSTF) {
            value &= ~CTRLSTS_FLAGS_MASK;
        }
        if (value & CTRLSTS_LSIEN) {
            value |= CTRLSTS_LSIRD;
        }
        break;
    default:
        break;
    }

    s->regs[addr >> 2] = value;
}

static const MemoryRegionOps n32g4fr_rcc_ops = {
    .read = n32g4fr_rcc_read,
    .write = n32g4fr_rcc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void n32g4fr_rcc_init(Object *obj)
{
    N32G4FRRccState *s = N32G4FR_RCC(obj);

    memory_region_init_io(&s->mmio, obj, &n32g4fr_rcc_ops, s,
                          TYPE_N32G4FR_RCC, N32G4FR_RCC_PERIPHERAL_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    qdev_init_gpio_in_named(DEVICE(obj), n32g4fr_rcc_iwdg_reset,
                            "iwdg-reset", 1);
}

static const VMStateDescription vmstate_n32g4fr_rcc = {
    .name = TYPE_N32G4FR_RCC,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, N32G4FRRccState, N32G4FR_RCC_NREGS),
        VMSTATE_UINT32(pending_rst_flags, N32G4FRRccState),
        VMSTATE_END_OF_LIST()
    }
};

static void n32g4fr_rcc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_n32g4fr_rcc;
    device_class_set_legacy_reset(dc, n32g4fr_rcc_reset);
}

static const TypeInfo n32g4fr_rcc_info = {
    .name          = TYPE_N32G4FR_RCC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N32G4FRRccState),
    .instance_init = n32g4fr_rcc_init,
    .class_init    = n32g4fr_rcc_class_init,
};

static void n32g4fr_rcc_register_types(void)
{
    type_register_static(&n32g4fr_rcc_info);
}

type_init(n32g4fr_rcc_register_types)
