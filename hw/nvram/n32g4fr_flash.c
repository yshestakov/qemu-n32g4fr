/*
 * N32G4FR embedded flash and flash interface
 *
 * The main flash array and its controller (N32G4FR User Manual V3.5.0
 * section 2.2): FLASH_KEY unlock sequence, 32 bit word programming
 * (FLASH_CTRL.PG), page and mass erase (PER/MER + START), and the status
 * flags and interrupt. Operations complete immediately, so BUSY never
 * reads as set.
 *
 * The flash array is a ROM device: reads go straight to memory, writes are
 * routed here and only take effect while programming is enabled. Erased
 * flash reads as 0xFF. With a "drive" (e.g. -drive if=pflash,format=raw)
 * the array is loaded from the backing file and every program/erase is
 * written back to it, so data the guest stores in flash persists.
 * Under record/replay the file is not written: a synchronous block write
 * from the vCPU thread would wait for the main loop, which waits for the
 * replay lock the vCPU holds. Snapshots still capture the array (RAM).
 *
 * Not modelled: option byte programming, read/write protection, the flash
 * encryption, ECC and the iCache.
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
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "system/replay.h"
#include "qemu/log.h"
#include "qemu/bitops.h"
#include "trace.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "system/block-backend.h"
#include "hw/nvram/n32g4fr_flash.h"

#define FLASH_AC        0x00
#define FLASH_KEY       0x04
#define FLASH_OPTKEY    0x08
#define FLASH_STS       0x0C
#define FLASH_CTRL      0x10
#define FLASH_ADD       0x14
#define FLASH_OB        0x1C
#define FLASH_WRP       0x20
#define FLASH_ECC       0x24
#define FLASH_RDN       0x2C
#define FLASH_CAHR      0x30

#define KEY1            0x45670123
#define KEY2            0xCDEF89AB

#define AC_PRFTBFE      BIT(4)
#define AC_PRFTBFS      BIT(5)
#define AC_ICAHRST      BIT(6)
#define AC_RESET        0x30

#define STS_PGERR       BIT(2)
#define STS_PVERR       BIT(3)
#define STS_WRPERR      BIT(4)
#define STS_EOP         BIT(5)
#define STS_EVERR       BIT(6)
#define STS_ECCERR      BIT(7)
#define STS_W1C         (STS_PGERR | STS_PVERR | STS_WRPERR | STS_EOP | \
                         STS_EVERR | STS_ECCERR)

#define CTRL_PG         BIT(0)
#define CTRL_PER        BIT(1)
#define CTRL_MER        BIT(2)
#define CTRL_OPTPG      BIT(4)
#define CTRL_OPTER      BIT(5)
#define CTRL_START      BIT(6)
#define CTRL_LOCK       BIT(7)
#define CTRL_SMPSEL     BIT(8)
#define CTRL_OPTWE      BIT(9)
#define CTRL_ERRITE     BIT(10)
#define CTRL_FERRITE    BIT(11)
#define CTRL_EOPITE     BIT(12)
#define CTRL_ECERRITE   BIT(13)
#define CTRL_MASK       0x3FF7
#define CTRL_RESET      CTRL_LOCK

#define OB_RESET        0x03FFFFFC
#define WRP_RESET       0xFFFFFFFF

/* Flash array addresses in FLASH_ADD are absolute */
#define FLASH_BASE      0x08000000

static void n32g4fr_flash_update_irq(N32G4FRFlashState *s)
{
    bool irq = ((s->sts & STS_EOP) && (s->ctrl & CTRL_EOPITE)) ||
               ((s->sts & (STS_PGERR | STS_WRPERR)) &&
                (s->ctrl & CTRL_ERRITE)) ||
               ((s->sts & (STS_PVERR | STS_EVERR)) &&
                (s->ctrl & CTRL_FERRITE)) ||
               ((s->sts & STS_ECCERR) && (s->ctrl & CTRL_ECERRITE));

    qemu_set_irq(s->irq, irq);
}

/* Write a range of the array back to the backing file, if any */
static void n32g4fr_flash_sync(N32G4FRFlashState *s, hwaddr offset,
                               hwaddr size)
{
    memory_region_flush_rom_device(&s->flash, offset, size);

    if (s->blk && replay_mode == REPLAY_MODE_NONE &&
        blk_pwrite(s->blk, offset, size, s->storage + offset, 0) < 0) {
        error_report("%s: failed to write backing file", __func__);
    }
}

static void n32g4fr_flash_erase(N32G4FRFlashState *s, hwaddr offset,
                                hwaddr size)
{
    trace_n32g4fr_flash_erase(offset, size);
    memset(s->storage + offset, 0xFF, size);
    n32g4fr_flash_sync(s, offset, size);
    s->sts |= STS_EOP;
}

static void n32g4fr_flash_start(N32G4FRFlashState *s)
{
    if (s->ctrl & CTRL_MER) {
        n32g4fr_flash_erase(s, 0, s->flash_size);
    } else if (s->ctrl & CTRL_PER) {
        hwaddr offset = (s->add - FLASH_BASE) &
                        ~(hwaddr)(N32G4FR_FLASH_PAGE_SIZE - 1);

        if (s->add < FLASH_BASE || offset >= s->flash_size) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: page erase of bad address "
                          "0x%08" PRIx32 "\n", __func__, s->add);
            s->sts |= STS_EVERR;
            return;
        }
        n32g4fr_flash_erase(s, offset, N32G4FR_FLASH_PAGE_SIZE);
    } else if (s->ctrl & (CTRL_OPTPG | CTRL_OPTER)) {
        qemu_log_mask(LOG_UNIMP, "%s: option bytes are not implemented\n",
                      __func__);
    }
}

static uint64_t n32g4fr_flash_read(void *opaque, hwaddr addr,
                                   unsigned int size)
{
    N32G4FRFlashState *s = N32G4FR_FLASH(opaque);
    uint32_t value = 0;

    switch (addr) {
    case FLASH_AC:
        value = s->ac;
        break;
    case FLASH_KEY:
    case FLASH_OPTKEY:
        /* Write-only */
        break;
    case FLASH_STS:
        value = s->sts;
        break;
    case FLASH_CTRL:
        value = s->ctrl;
        break;
    case FLASH_ADD:
        value = s->add;
        break;
    case FLASH_OB:
        value = s->ob;
        break;
    case FLASH_WRP:
        value = s->wrp;
        break;
    case FLASH_ECC:
    case FLASH_RDN:
    case FLASH_CAHR:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        break;
    }

    trace_n32g4fr_flash_reg_read(addr, value);
    return value;
}

static void n32g4fr_flash_key(N32G4FRFlashState *s, uint8_t *state,
                              uint32_t value, uint32_t unlock_bit)
{
    if (*state == 0 && value == KEY1) {
        *state = 1;
        return;
    }
    if (*state == 1 && value == KEY2) {
        *state = 0;
        if (unlock_bit == CTRL_LOCK) {
            s->ctrl &= ~CTRL_LOCK;
        } else {
            s->ctrl |= unlock_bit;
        }
        return;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: bad unlock sequence, flash locked "
                  "until reset\n", __func__);
    *state = 0;
    s->key_fault = true;
    s->ctrl |= CTRL_LOCK;
}

static void n32g4fr_flash_write(void *opaque, hwaddr addr,
                                uint64_t val64, unsigned int size)
{
    N32G4FRFlashState *s = N32G4FR_FLASH(opaque);
    uint32_t value = val64;

    trace_n32g4fr_flash_reg_write(addr, value);

    switch (addr) {
    case FLASH_AC:
        /* Prefetch status follows enable; iCache reset is self-clearing */
        value &= ~(AC_PRFTBFS | AC_ICAHRST);
        if (value & AC_PRFTBFE) {
            value |= AC_PRFTBFS;
        }
        s->ac = value & 0xF7;
        break;
    case FLASH_KEY:
        if (!s->key_fault) {
            n32g4fr_flash_key(s, &s->key_state, value, CTRL_LOCK);
        }
        break;
    case FLASH_OPTKEY:
        if (!s->key_fault && !(s->ctrl & CTRL_LOCK)) {
            n32g4fr_flash_key(s, &s->optkey_state, value, CTRL_OPTWE);
        }
        break;
    case FLASH_STS:
        s->sts &= ~(value & STS_W1C);
        break;
    case FLASH_CTRL:
        if (s->ctrl & CTRL_LOCK) {
            /* Only re-locking is possible while locked */
            break;
        }
        /* OPTWE can only be cleared by software */
        s->ctrl = (value & CTRL_MASK & ~(CTRL_START | CTRL_OPTWE)) |
                  (s->ctrl & value & CTRL_OPTWE);
        if (value & CTRL_START) {
            n32g4fr_flash_start(s);
        }
        break;
    case FLASH_ADD:
        s->add = value;
        break;
    case FLASH_OB:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: FLASH_OB is read-only\n",
                      __func__);
        break;
    case FLASH_WRP:
    case FLASH_ECC:
    case FLASH_RDN:
    case FLASH_CAHR:
        qemu_log_mask(LOG_UNIMP, "%s: register 0x%" HWADDR_PRIx
                      " not implemented\n", __func__, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        return;
    }

    n32g4fr_flash_update_irq(s);
}

static const MemoryRegionOps n32g4fr_flash_reg_ops = {
    .read = n32g4fr_flash_read,
    .write = n32g4fr_flash_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static MemTxResult n32g4fr_flash_array_read(void *opaque, hwaddr offset,
                                            uint64_t *data, unsigned size,
                                            MemTxAttrs attrs)
{
    /*
     * The array stays in ROMD mode, so reads go straight to memory and
     * never come here.
     */
    g_assert_not_reached();
}

static MemTxResult n32g4fr_flash_array_write(void *opaque, hwaddr offset,
                                             uint64_t value, unsigned size,
                                             MemTxAttrs attrs)
{
    N32G4FRFlashState *s = N32G4FR_FLASH(opaque);
    uint32_t old;

    if (size != 4) {
        /* Only whole words can be programmed; anything else bus-faults */
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %u byte write to flash 0x%"
                      HWADDR_PRIx "\n", __func__, size, offset);
        return MEMTX_ERROR;
    }
    if ((s->ctrl & CTRL_LOCK) || !(s->ctrl & CTRL_PG)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to flash 0x%" HWADDR_PRIx
                      " while programming is not enabled\n", __func__, offset);
        s->sts |= STS_PGERR;
        n32g4fr_flash_update_irq(s);
        return MEMTX_OK;
    }

    old = ldl_le_p(s->storage + offset);
    if (!(s->ctrl & CTRL_SMPSEL) && old != 0xFFFFFFFF) {
        /* SMP1 mode refuses to program a word that is not erased */
        s->sts |= STS_PGERR;
    } else {
        trace_n32g4fr_flash_program(offset, value);
        /* NOR flash can only clear bits */
        stl_le_p(s->storage + offset, old & value);
        n32g4fr_flash_sync(s, offset, 4);
        s->sts |= STS_EOP;
    }
    n32g4fr_flash_update_irq(s);
    return MEMTX_OK;
}

static const MemoryRegionOps n32g4fr_flash_array_ops = {
    .read_with_attrs = n32g4fr_flash_array_read,
    .write_with_attrs = n32g4fr_flash_array_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void n32g4fr_flash_reset(DeviceState *dev)
{
    N32G4FRFlashState *s = N32G4FR_FLASH(dev);

    s->ac = AC_RESET;
    s->sts = 0;
    s->ctrl = CTRL_RESET;
    s->add = 0;
    s->ob = OB_RESET;
    s->wrp = WRP_RESET;
    s->key_state = 0;
    s->optkey_state = 0;
    s->key_fault = false;
    n32g4fr_flash_update_irq(s);
}

static void n32g4fr_flash_init(Object *obj)
{
    N32G4FRFlashState *s = N32G4FR_FLASH(obj);

    memory_region_init_io(&s->mmio, obj, &n32g4fr_flash_reg_ops, s,
                          "n32g4fr.flash-if", 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static void n32g4fr_flash_realize(DeviceState *dev, Error **errp)
{
    N32G4FRFlashState *s = N32G4FR_FLASH(dev);

    if (!memory_region_init_rom_device(&s->flash, OBJECT(dev),
                                       &n32g4fr_flash_array_ops, s,
                                       "N32G4FR.flash", s->flash_size,
                                       errp)) {
        return;
    }
    s->storage = memory_region_get_ram_ptr(&s->flash);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->flash);

    if (!s->blk) {
        memset(s->storage, 0xFF, s->flash_size);
        return;
    }
    if (blk_getlength(s->blk) != s->flash_size) {
        error_setg(errp, "flash backing file must be %" PRIu32 " bytes",
                   s->flash_size);
        return;
    }
    if (blk_set_perm(s->blk, BLK_PERM_CONSISTENT_READ | BLK_PERM_WRITE,
                     BLK_PERM_ALL, errp) < 0) {
        return;
    }
    if (blk_pread(s->blk, 0, s->flash_size, s->storage, 0) < 0) {
        error_setg(errp, "failed to read flash backing file");
        return;
    }
}

static const VMStateDescription vmstate_n32g4fr_flash = {
    .name = TYPE_N32G4FR_FLASH,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ac, N32G4FRFlashState),
        VMSTATE_UINT32(sts, N32G4FRFlashState),
        VMSTATE_UINT32(ctrl, N32G4FRFlashState),
        VMSTATE_UINT32(add, N32G4FRFlashState),
        VMSTATE_UINT32(ob, N32G4FRFlashState),
        VMSTATE_UINT32(wrp, N32G4FRFlashState),
        VMSTATE_UINT8(key_state, N32G4FRFlashState),
        VMSTATE_UINT8(optkey_state, N32G4FRFlashState),
        VMSTATE_BOOL(key_fault, N32G4FRFlashState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property n32g4fr_flash_properties[] = {
    DEFINE_PROP_UINT32("flash-size", N32G4FRFlashState, flash_size,
                       512 * 1024),
    DEFINE_PROP_DRIVE("drive", N32G4FRFlashState, blk),
};

static void n32g4fr_flash_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = n32g4fr_flash_realize;
    dc->vmsd = &vmstate_n32g4fr_flash;
    device_class_set_legacy_reset(dc, n32g4fr_flash_reset);
    device_class_set_props(dc, n32g4fr_flash_properties);
}

static const TypeInfo n32g4fr_flash_info = {
    .name          = TYPE_N32G4FR_FLASH,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N32G4FRFlashState),
    .instance_init = n32g4fr_flash_init,
    .class_init    = n32g4fr_flash_class_init,
};

static void n32g4fr_flash_register_types(void)
{
    type_register_static(&n32g4fr_flash_info);
}

type_init(n32g4fr_flash_register_types)
