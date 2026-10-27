/*
 * N32G4FR DMA controller
 *
 * Eight channel STM32F1-style DMA with an N32 specific per channel request
 * selection (DMA_CHSELx / DMA_CHMAPEN), see N32G4FR User Manual V3.5.0
 * chapter 8.
 *
 * Peripherals signal their DMA requests on the GPIO inputs of this device,
 * numbered by request source (the DMA_CHSELx.CH_SEL value). A channel
 * moves one data item for each pass in which its request is asserted, so
 * peripherals must lower the request once the item has been consumed
 * (e.g. a USART lowers RX request when its data register is read).
 * Memory to memory channels run to completion as soon as they are enabled.
 * Bus timing and arbitration latency are not modelled.
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
#include "qemu/main-loop.h"
#include "qapi/error.h"
#include "trace.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "hw/dma/n32g4fr_dma.h"

#define DMA_INTSTS          0x00
#define DMA_INTCLR          0x04
#define DMA_CH_BASE         0x08
#define DMA_CH_STRIDE       0x14
#define DMA_CHMAPEN         0xA8

/* Channel register offsets relative to DMA_CH_BASE + DMA_CH_STRIDE * ch */
#define DMA_CHCFG           0x00
#define DMA_TXNUM           0x04
#define DMA_PADDR           0x08
#define DMA_MADDR           0x0C
#define DMA_CHSEL           0x10

/* DMA_INTSTS / DMA_INTCLR, per channel nibble */
#define INT_GLBF            BIT(0)
#define INT_TXCF            BIT(1)
#define INT_HTXF            BIT(2)
#define INT_ERRF            BIT(3)

/* DMA_CHCFGx */
#define CHCFG_CHEN          BIT(0)
#define CHCFG_TXCIE         BIT(1)
#define CHCFG_HTXIE         BIT(2)
#define CHCFG_ERRIE         BIT(3)
#define CHCFG_DIR           BIT(4)
#define CHCFG_CIRC          BIT(5)
#define CHCFG_PINC          BIT(6)
#define CHCFG_MINC          BIT(7)
#define CHCFG_PSIZE_SHIFT   8
#define CHCFG_MSIZE_SHIFT   10
#define CHCFG_PRIOLVL_SHIFT 12
#define CHCFG_MEM2MEM       BIT(14)
#define CHCFG_MASK          0x7FFF

#define TXNUM_MASK          0xFFFF
#define CHSEL_MASK          0x3F
#define CHMAPEN_MAP_EN      BIT(0)

/* Upper bound of items moved per pass before yielding to the main loop */
#define DMA_MAX_ITEMS_PER_PASS 0x20000

#define REQ_RANGE(lo, hi)   MAKE_64BIT_MASK(lo, (hi) - (lo) + 1)

/*
 * Fixed request to channel mapping, used while DMA_CHMAPEN.MAP_EN = 0.
 * UM Table 8-4/8-5; the DMA1 CH_SEL numbering is grouped by channel.
 */
static const uint64_t dma1_fixed_map[N32G4FR_DMA_NUM_CHANNELS] = {
    /* ADC1 UART5_TX I2C3_TX TIM2_CH3 TIM4_CH1 */
    REQ_RANGE(0, 4),
    /* USART3_TX I2C3_RX TIM1_CH1 TIM2_UP TIM3_CH3 SPI1_RX */
    REQ_RANGE(5, 10),
    /* USART3_RX TIM1_CH2 TIM3_CH4 TIM3_UP SPI1_TX */
    REQ_RANGE(11, 15),
    /* USART1_TX TIM1_CH4/TRIG/COM TIM4_CH2 SPI2_RX I2C2_TX */
    REQ_RANGE(16, 22),
    /* USART1_RX TIM1_UP SPI2_TX TIM2_CH1 TIM4_CH3 I2C2_RX */
    REQ_RANGE(23, 28),
    /* USART2_RX TIM1_CH3 TIM3_CH1 TIM3_TRIG I2C1_TX */
    REQ_RANGE(29, 33),
    /* USART2_TX TIM2_CH2 TIM2_CH4 TIM4_UP I2C1_RX */
    REQ_RANGE(34, 38),
    /* ADC2 UART5_RX */
    REQ_RANGE(39, 40),
};

static const uint64_t dma2_fixed_map[N32G4FR_DMA_NUM_CHANNELS] = {
    /* TIM5_CH4/TRIG TIM8_CH3/UP SPI3_RX UART6_RX */
    REQ_RANGE(0, 5),
    /* TIM8_CH4/TRIG/COM SPI3_TX UART6_TX */
    REQ_RANGE(6, 8) | REQ_RANGE(11, 12),
    /* TIM5_CH3/UP TIM8_CH1 UART4_RX TIM6_UP DAC1 */
    REQ_RANGE(9, 10) | REQ_RANGE(13, 16),
    /* TIM5_CH2 SDIO TIM7_UP DAC2 */
    REQ_RANGE(17, 20),
    /* TIM8_CH2 TIM5_CH1 UART4_TX */
    REQ_RANGE(22, 24),
    /* QSPI_RX I2C4_TX UART7_RX */
    REQ_RANGE(25, 27),
    /* QSPI_TX I2C4_RX UART7_TX */
    REQ_RANGE(28, 30),
    /* DVP */
    BIT_ULL(32),
};

static void n32g4fr_dma_update_irq(N32G4FRDmaState *s, int ch)
{
    uint32_t flags = extract32(s->intsts, ch * 4, 4);
    uint32_t enables = s->ch[ch].chcfg & (CHCFG_TXCIE | CHCFG_HTXIE |
                                          CHCFG_ERRIE);

    /* TXCF/HTXF/ERRF line up with TXCIE/HTXIE/ERRIE */
    qemu_set_irq(s->irq[ch], !!(flags & enables));
}

static void n32g4fr_dma_set_flags(N32G4FRDmaState *s, int ch, uint32_t flags)
{
    s->intsts |= (flags | INT_GLBF) << (ch * 4);
    n32g4fr_dma_update_irq(s, ch);
}

static bool n32g4fr_dma_requested(N32G4FRDmaState *s, int ch)
{
    if (s->chmapen & CHMAPEN_MAP_EN) {
        return extract64(s->drq, s->ch[ch].chsel, 1);
    }
    if (s->controller == 2) {
        return s->drq & dma2_fixed_map[ch];
    }
    return s->drq & dma1_fixed_map[ch];
}

static bool n32g4fr_dma_ready(N32G4FRDmaState *s, int ch)
{
    N32G4FRDmaChannel *c = &s->ch[ch];

    if (!(c->chcfg & CHCFG_CHEN) || c->txnum == 0) {
        return false;
    }
    return (c->chcfg & CHCFG_MEM2MEM) || n32g4fr_dma_requested(s, ch);
}

/* Highest priority ready channel: PRIOLVL first, then lowest number */
static int n32g4fr_dma_pick(N32G4FRDmaState *s)
{
    int best = -1;
    int best_prio = -1;

    for (int ch = 0; ch < N32G4FR_DMA_NUM_CHANNELS; ch++) {
        int prio = extract32(s->ch[ch].chcfg, CHCFG_PRIOLVL_SHIFT, 2);

        if (n32g4fr_dma_ready(s, ch) && prio > best_prio) {
            best = ch;
            best_prio = prio;
        }
    }
    return best;
}

static MemTxResult n32g4fr_dma_load(N32G4FRDmaState *s, hwaddr addr,
                                    int size, uint32_t *val)
{
    MemTxResult res = MEMTX_OK;

    switch (size) {
    case 1:
        *val = address_space_ldub(&s->downstream_as, addr,
                                  MEMTXATTRS_UNSPECIFIED, &res);
        break;
    case 2:
        *val = address_space_lduw_le(&s->downstream_as, addr,
                                     MEMTXATTRS_UNSPECIFIED, &res);
        break;
    default:
        *val = address_space_ldl_le(&s->downstream_as, addr,
                                    MEMTXATTRS_UNSPECIFIED, &res);
        break;
    }
    return res;
}

static MemTxResult n32g4fr_dma_store(N32G4FRDmaState *s, hwaddr addr,
                                     int size, uint32_t val)
{
    MemTxResult res = MEMTX_OK;

    switch (size) {
    case 1:
        address_space_stb(&s->downstream_as, addr, val,
                          MEMTXATTRS_UNSPECIFIED, &res);
        break;
    case 2:
        address_space_stw_le(&s->downstream_as, addr, val,
                             MEMTXATTRS_UNSPECIFIED, &res);
        break;
    default:
        address_space_stl_le(&s->downstream_as, addr, val,
                             MEMTXATTRS_UNSPECIFIED, &res);
        break;
    }
    return res;
}

static int n32g4fr_dma_size(N32G4FRDmaState *s, int ch, int shift)
{
    int code = extract32(s->ch[ch].chcfg, shift, 2);

    if (code == 3) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: channel %d: reserved data size\n", __func__, ch + 1);
        code = 2;
    }
    return 1 << code;
}

/* Move one data item on channel ch */
static void n32g4fr_dma_transfer_one(N32G4FRDmaState *s, int ch)
{
    N32G4FRDmaChannel *c = &s->ch[ch];
    int psize = n32g4fr_dma_size(s, ch, CHCFG_PSIZE_SHIFT);
    int msize = n32g4fr_dma_size(s, ch, CHCFG_MSIZE_SHIFT);
    /* PADDR/MADDR ignore the low address bits for wider accesses */
    hwaddr paddr = c->cur_paddr & ~(psize - 1);
    hwaddr maddr = c->cur_maddr & ~(msize - 1);
    uint32_t val = 0;
    MemTxResult res;

    /* Narrower destinations get the low bits, wider ones are zero-extended */
    if (c->chcfg & CHCFG_DIR) {
        res = n32g4fr_dma_load(s, maddr, msize, &val);
        if (res == MEMTX_OK) {
            res = n32g4fr_dma_store(s, paddr, psize, val);
        }
    } else {
        res = n32g4fr_dma_load(s, paddr, psize, &val);
        if (res == MEMTX_OK) {
            res = n32g4fr_dma_store(s, maddr, msize, val);
        }
    }

    trace_n32g4fr_dma_transfer(s->controller, ch + 1, paddr, maddr, val,
                               c->txnum);

    if (res != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: DMA%d channel %d: bus error (paddr 0x%"
                      HWADDR_PRIx " maddr 0x%" HWADDR_PRIx ")\n",
                      __func__, s->controller, ch + 1, paddr, maddr);
        c->chcfg &= ~CHCFG_CHEN;
        n32g4fr_dma_set_flags(s, ch, INT_ERRF);
        return;
    }

    if (c->chcfg & CHCFG_PINC) {
        c->cur_paddr += psize;
    }
    if (c->chcfg & CHCFG_MINC) {
        c->cur_maddr += msize;
    }
    c->txnum--;

    if (c->txnum == c->txnum_reload / 2 && c->txnum != 0) {
        n32g4fr_dma_set_flags(s, ch, INT_HTXF);
    }
    if (c->txnum == 0) {
        uint32_t flags = INT_TXCF;

        if (c->txnum_reload == 1) {
            flags |= INT_HTXF;
        }
        if (c->chcfg & CHCFG_CIRC) {
            c->txnum = c->txnum_reload;
            c->cur_paddr = c->paddr;
            c->cur_maddr = c->maddr;
        }
        n32g4fr_dma_set_flags(s, ch, flags);
    }
}

static void n32g4fr_dma_process(N32G4FRDmaState *s)
{
    int ch;

    if (s->busy) {
        /* Re-entered from a request change during a transfer */
        return;
    }
    s->busy = true;

    for (int n = 0; n < DMA_MAX_ITEMS_PER_PASS; n++) {
        ch = n32g4fr_dma_pick(s);
        if (ch < 0) {
            s->busy = false;
            return;
        }
        n32g4fr_dma_transfer_one(s, ch);
    }

    /* A request that never drops (e.g. circular TX): yield, resume later */
    s->busy = false;
    qemu_bh_schedule(s->bh);
}

static void n32g4fr_dma_bh(void *opaque)
{
    n32g4fr_dma_process(N32G4FR_DMA(opaque));
}

static void n32g4fr_dma_set_request(void *opaque, int n, int level)
{
    N32G4FRDmaState *s = N32G4FR_DMA(opaque);
    uint64_t old = s->drq;

    s->drq = deposit64(s->drq, n, 1, !!level);
    if (s->drq != old) {
        trace_n32g4fr_dma_request(s->controller, n, !!level);
    }
    /*
     * Requests usually change inside the requesting peripheral's MMIO
     * handler; touching that peripheral again from here would be blocked
     * as re-entrant I/O, so service them from a bottom half.
     */
    if (level && !s->busy) {
        qemu_bh_schedule(s->bh);
    }
}

static void n32g4fr_dma_reset(DeviceState *dev)
{
    N32G4FRDmaState *s = N32G4FR_DMA(dev);

    s->intsts = 0;
    s->chmapen = 0;
    memset(s->ch, 0, sizeof(s->ch));
    for (int ch = 0; ch < N32G4FR_DMA_NUM_CHANNELS; ch++) {
        n32g4fr_dma_update_irq(s, ch);
    }
}

static uint64_t n32g4fr_dma_read(void *opaque, hwaddr addr, unsigned int size)
{
    N32G4FRDmaState *s = N32G4FR_DMA(opaque);
    uint32_t value = 0;

    if (addr == DMA_INTSTS) {
        value = s->intsts;
    } else if (addr == DMA_INTCLR) {
        value = 0;
    } else if (addr == DMA_CHMAPEN) {
        value = s->chmapen;
    } else if (addr >= DMA_CH_BASE &&
               addr < DMA_CH_BASE + DMA_CH_STRIDE * N32G4FR_DMA_NUM_CHANNELS) {
        int ch = (addr - DMA_CH_BASE) / DMA_CH_STRIDE;
        N32G4FRDmaChannel *c = &s->ch[ch];

        switch ((addr - DMA_CH_BASE) % DMA_CH_STRIDE) {
        case DMA_CHCFG:
            value = c->chcfg;
            break;
        case DMA_TXNUM:
            value = c->txnum;
            break;
        case DMA_PADDR:
            value = c->paddr;
            break;
        case DMA_MADDR:
            value = c->maddr;
            break;
        case DMA_CHSEL:
            value = c->chsel;
            break;
        }
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
    }

    trace_n32g4fr_dma_read(s->controller, addr, value);
    return value;
}

static void n32g4fr_dma_write_channel(N32G4FRDmaState *s, int ch, hwaddr reg,
                                      uint32_t value)
{
    N32G4FRDmaChannel *c = &s->ch[ch];
    bool enabled = c->chcfg & CHCFG_CHEN;

    if (reg != DMA_CHCFG && reg != DMA_CHSEL && enabled) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: DMA%d channel %d: register 0x%" HWADDR_PRIx
                      " written while enabled\n",
                      __func__, s->controller, ch + 1, reg);
        return;
    }

    switch (reg) {
    case DMA_CHCFG:
        c->chcfg = value & CHCFG_MASK;
        if (!enabled && (c->chcfg & CHCFG_CHEN)) {
            c->cur_paddr = c->paddr;
            c->cur_maddr = c->maddr;
        }
        n32g4fr_dma_update_irq(s, ch);
        break;
    case DMA_TXNUM:
        c->txnum = value & TXNUM_MASK;
        c->txnum_reload = c->txnum;
        break;
    case DMA_PADDR:
        c->paddr = value;
        break;
    case DMA_MADDR:
        c->maddr = value;
        break;
    case DMA_CHSEL:
        if (s->chmapen & CHMAPEN_MAP_EN) {
            c->chsel = value & CHSEL_MASK;
        }
        break;
    }
}

static void n32g4fr_dma_write(void *opaque, hwaddr addr,
                              uint64_t val64, unsigned int size)
{
    N32G4FRDmaState *s = N32G4FR_DMA(opaque);
    uint32_t value = val64;

    trace_n32g4fr_dma_write(s->controller, addr, value);

    if (addr == DMA_INTSTS) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: DMA_INTSTS is read-only\n",
                      __func__);
        return;
    } else if (addr == DMA_INTCLR) {
        for (int ch = 0; ch < N32G4FR_DMA_NUM_CHANNELS; ch++) {
            uint32_t clr = extract32(value, ch * 4, 4);

            /* As on STM32F1, clearing GLBF clears all flags of the channel */
            if (clr & INT_GLBF) {
                clr = 0xF;
            }
            s->intsts &= ~(clr << (ch * 4));
            n32g4fr_dma_update_irq(s, ch);
        }
        return;
    } else if (addr == DMA_CHMAPEN) {
        if ((value ^ s->chmapen) & CHMAPEN_MAP_EN) {
            /* Enabling the map resets the selection registers */
            for (int ch = 0; ch < N32G4FR_DMA_NUM_CHANNELS; ch++) {
                s->ch[ch].chsel = 0;
            }
        }
        s->chmapen = value & CHMAPEN_MAP_EN;
    } else if (addr >= DMA_CH_BASE &&
               addr < DMA_CH_BASE + DMA_CH_STRIDE * N32G4FR_DMA_NUM_CHANNELS) {
        n32g4fr_dma_write_channel(s, (addr - DMA_CH_BASE) / DMA_CH_STRIDE,
                                  (addr - DMA_CH_BASE) % DMA_CH_STRIDE, value);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        return;
    }

    n32g4fr_dma_process(s);
}

static const MemoryRegionOps n32g4fr_dma_ops = {
    .read = n32g4fr_dma_read,
    .write = n32g4fr_dma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void n32g4fr_dma_init(Object *obj)
{
    N32G4FRDmaState *s = N32G4FR_DMA(obj);

    memory_region_init_io(&s->mmio, obj, &n32g4fr_dma_ops, s,
                          TYPE_N32G4FR_DMA, 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);

    for (int ch = 0; ch < N32G4FR_DMA_NUM_CHANNELS; ch++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[ch]);
    }
    qdev_init_gpio_in(DEVICE(obj), n32g4fr_dma_set_request,
                      N32G4FR_DMA_NUM_REQUESTS);
}

static void n32g4fr_dma_realize(DeviceState *dev, Error **errp)
{
    N32G4FRDmaState *s = N32G4FR_DMA(dev);

    if (!s->downstream) {
        error_setg(errp, "N32G4FR DMA 'downstream' link not set");
        return;
    }
    if (s->controller != 1 && s->controller != 2) {
        error_setg(errp, "N32G4FR DMA 'controller' must be 1 or 2");
        return;
    }

    address_space_init(&s->downstream_as, s->downstream,
                       "n32g4fr-dma-downstream");
    s->bh = qemu_bh_new_guarded(n32g4fr_dma_bh, s,
                                &dev->mem_reentrancy_guard);
}

static const VMStateDescription vmstate_n32g4fr_dma_channel = {
    .name = "n32g4fr.dma/channel",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(chcfg, N32G4FRDmaChannel),
        VMSTATE_UINT32(txnum, N32G4FRDmaChannel),
        VMSTATE_UINT32(txnum_reload, N32G4FRDmaChannel),
        VMSTATE_UINT32(paddr, N32G4FRDmaChannel),
        VMSTATE_UINT32(maddr, N32G4FRDmaChannel),
        VMSTATE_UINT32(chsel, N32G4FRDmaChannel),
        VMSTATE_UINT32(cur_paddr, N32G4FRDmaChannel),
        VMSTATE_UINT32(cur_maddr, N32G4FRDmaChannel),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_n32g4fr_dma = {
    .name = TYPE_N32G4FR_DMA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(intsts, N32G4FRDmaState),
        VMSTATE_UINT32(chmapen, N32G4FRDmaState),
        VMSTATE_STRUCT_ARRAY(ch, N32G4FRDmaState, N32G4FR_DMA_NUM_CHANNELS,
                             1, vmstate_n32g4fr_dma_channel,
                             N32G4FRDmaChannel),
        VMSTATE_UINT64(drq, N32G4FRDmaState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property n32g4fr_dma_properties[] = {
    DEFINE_PROP_LINK("downstream", N32G4FRDmaState, downstream,
                     TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_UINT8("controller", N32G4FRDmaState, controller, 1),
};

static void n32g4fr_dma_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = n32g4fr_dma_realize;
    dc->vmsd = &vmstate_n32g4fr_dma;
    device_class_set_legacy_reset(dc, n32g4fr_dma_reset);
    device_class_set_props(dc, n32g4fr_dma_properties);
}

static const TypeInfo n32g4fr_dma_info = {
    .name          = TYPE_N32G4FR_DMA,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N32G4FRDmaState),
    .instance_init = n32g4fr_dma_init,
    .class_init    = n32g4fr_dma_class_init,
};

static void n32g4fr_dma_register_types(void)
{
    type_register_static(&n32g4fr_dma_info);
}

type_init(n32g4fr_dma_register_types)
