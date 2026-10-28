/*
 * N32G4FR independent watchdog (IWDG)
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

/*
 * N32G4FR User Manual V3.5.0 chapter 16.  The 12-bit down counter runs from
 * the 40 kHz LSI; it is modelled as a timer on the virtual clock, which
 * stops while the VM is halted (e.g. by GDB), so like DBG_CTRL.IWDG_STOP
 * the watchdog never fires at a breakpoint.  Prescaler/reload updates
 * complete instantly (STS always reads 0).  The "reset-enable" input is
 * PWR_CTRL2.IWDGRSTEN: while it is low an expiry does not reset the chip
 * and the counter starts again.  The hardware watchdog option byte
 * (WDG_SW) is not modelled.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "system/watchdog.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "hw/watchdog/n32g4fr_iwdg.h"
#include "trace.h"

#define IWDG_LSI_HZ         40000

/* IWDG_KEY.KEYV values */
#define KEY_RELOAD          0xAAAA
#define KEY_UNLOCK          0x5555
#define KEY_START           0xCCCC

#define PREDIV_PD_MASK      0x7
#define RELV_REL_MASK       0xFFF
#define RELV_RESET          0xFFF

/* Prescaler /4 << PD, PD >= 6 is /256 */
static unsigned n32g4fr_iwdg_div(N32G4FRIwdgState *s)
{
    return 4 << MIN(s->prediv, 6);
}

static void n32g4fr_iwdg_arm(N32G4FRIwdgState *s)
{
    int64_t ns = muldiv64((uint64_t)(s->count + 1) * n32g4fr_iwdg_div(s),
                          NANOSECONDS_PER_SECOND, IWDG_LSI_HZ);

    trace_n32g4fr_iwdg_arm(s->count, n32g4fr_iwdg_div(s), ns / SCALE_MS);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns);
}

static void n32g4fr_iwdg_expired(void *opaque)
{
    N32G4FRIwdgState *s = N32G4FR_IWDG(opaque);

    trace_n32g4fr_iwdg_expired();
    qemu_log_mask(CPU_LOG_RESET, "N32G4FR IWDG expired%s\n",
                  s->reset_enable ? "" : " (reset disabled by PWR)");
    if (!s->reset_enable) {
        s->count = RELV_RESET;
        n32g4fr_iwdg_arm(s);
        return;
    }
    if (get_watchdog_action() == WATCHDOG_ACTION_RESET) {
        /* Latch RCC_CTRLSTS.IWDGRSTF across the reset that follows */
        qemu_irq_pulse(s->reset_flag);
    } else {
        /* No reset: keep counting so it fires again */
        s->count = RELV_RESET;
        n32g4fr_iwdg_arm(s);
    }
    watchdog_perform_action();
}

static void n32g4fr_iwdg_set_reset_enable(void *opaque, int n, int level)
{
    N32G4FRIwdgState *s = N32G4FR_IWDG(opaque);

    s->reset_enable = level;
}

static void n32g4fr_iwdg_reset(DeviceState *dev)
{
    N32G4FRIwdgState *s = N32G4FR_IWDG(dev);

    timer_del(s->timer);
    s->prediv = 0;
    s->relv = RELV_RESET;
    s->count = RELV_RESET;
    s->unlocked = false;
    s->running = false;
}

static uint64_t n32g4fr_iwdg_read(void *opaque, hwaddr addr, unsigned int size)
{
    N32G4FRIwdgState *s = N32G4FR_IWDG(opaque);
    uint32_t value = 0;

    switch (addr) {
    case N32G4FR_IWDG_KEY:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: IWDG_KEY is write-only\n",
                      __func__);
        break;
    case N32G4FR_IWDG_PREDIV:
        value = s->prediv;
        break;
    case N32G4FR_IWDG_RELV:
        value = s->relv;
        break;
    case N32G4FR_IWDG_STS:
        /* PVU/CRVU: updates complete instantly */
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        break;
    }
    trace_n32g4fr_iwdg_read(addr, value);
    return value;
}

static void n32g4fr_iwdg_write(void *opaque, hwaddr addr,
                               uint64_t val64, unsigned int size)
{
    N32G4FRIwdgState *s = N32G4FR_IWDG(opaque);
    uint32_t value = val64;

    trace_n32g4fr_iwdg_write(addr, value);

    switch (addr) {
    case N32G4FR_IWDG_KEY:
        value &= 0xFFFF;
        /* Any key other than 0x5555 write-protects PREDIV/RELV again */
        s->unlocked = value == KEY_UNLOCK;
        switch (value) {
        case KEY_RELOAD:
            s->count = s->relv;
            if (s->running) {
                n32g4fr_iwdg_arm(s);
            }
            break;
        case KEY_START:
            if (!s->running) {
                s->running = true;
                n32g4fr_iwdg_arm(s);
            }
            break;
        }
        break;
    case N32G4FR_IWDG_PREDIV:
    case N32G4FR_IWDG_RELV:
        if (!s->unlocked) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: write to protected register 0x%"HWADDR_PRIx
                          "\n", __func__, addr);
            break;
        }
        /* Takes effect at the next reload */
        if (addr == N32G4FR_IWDG_PREDIV) {
            s->prediv = value & PREDIV_PD_MASK;
        } else {
            s->relv = value & RELV_REL_MASK;
        }
        break;
    case N32G4FR_IWDG_STS:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        break;
    }
}

static const MemoryRegionOps n32g4fr_iwdg_ops = {
    .read = n32g4fr_iwdg_read,
    .write = n32g4fr_iwdg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void n32g4fr_iwdg_init(Object *obj)
{
    N32G4FRIwdgState *s = N32G4FR_IWDG(obj);

    memory_region_init_io(&s->mmio, obj, &n32g4fr_iwdg_ops, s,
                          TYPE_N32G4FR_IWDG, N32G4FR_IWDG_PERIPHERAL_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    qdev_init_gpio_out_named(DEVICE(obj), &s->reset_flag, "reset-flag", 1);
    qdev_init_gpio_in_named(DEVICE(obj), n32g4fr_iwdg_set_reset_enable,
                            "reset-enable", 1);
    /* PWR drives this; enabled when nothing is connected */
    s->reset_enable = true;
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, n32g4fr_iwdg_expired, s);
}

static const VMStateDescription vmstate_n32g4fr_iwdg = {
    .name = TYPE_N32G4FR_IWDG,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, N32G4FRIwdgState),
        VMSTATE_UINT32(prediv, N32G4FRIwdgState),
        VMSTATE_UINT32(relv, N32G4FRIwdgState),
        VMSTATE_UINT32(count, N32G4FRIwdgState),
        VMSTATE_BOOL(unlocked, N32G4FRIwdgState),
        VMSTATE_BOOL(running, N32G4FRIwdgState),
        VMSTATE_BOOL(reset_enable, N32G4FRIwdgState),
        VMSTATE_END_OF_LIST()
    }
};

static void n32g4fr_iwdg_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_n32g4fr_iwdg;
    device_class_set_legacy_reset(dc, n32g4fr_iwdg_reset);
    set_bit(DEVICE_CATEGORY_WATCHDOG, dc->categories);
}

static const TypeInfo n32g4fr_iwdg_info = {
    .name          = TYPE_N32G4FR_IWDG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N32G4FRIwdgState),
    .instance_init = n32g4fr_iwdg_init,
    .class_init    = n32g4fr_iwdg_class_init,
};

static void n32g4fr_iwdg_register_types(void)
{
    type_register_static(&n32g4fr_iwdg_info);
}

type_init(n32g4fr_iwdg_register_types)
