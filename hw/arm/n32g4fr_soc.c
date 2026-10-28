/*
 * Nationstech N32G4FR SoC
 *
 * Based on the STM32F405 SoC model,
 * Copyright (c) 2014 Alistair Francis <alistair@alistair23.me>
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
#include "system/address-spaces.h"
#include "system/system.h"
#include "hw/arm/n32g4fr_soc.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/unimp.h"

/*
 * Addresses and IRQ numbers are from the N32G4FR Datasheet V2.5.0 memory
 * map, the Nationstech N32G4FR.svd and User Manual V3.5.0 (Table 7-1
 * vector table, Tables 8-4/8-5 DMA request mapping). The timers are
 * register compatible with the STM32F2xx model.
 */
#define RCC_ADDR                       0x40021000
#define IWDG_ADDR                      0x40003000
#define FLASH_IF_ADDR                  0x40022000
#define FLASH_IRQ                      4
#define USBD_ADDR                      0x40005C00
#define USBD_PMA_ADDR                  0x40006000
#define USB_HP_IRQ                     19
#define USB_LP_IRQ                     20
static const uint32_t dma_addr[] = { 0x40020000, 0x40020400 };
static const int dma_irq[N32_NUM_DMAS][N32G4FR_DMA_NUM_CHANNELS] = {
    { 11, 12, 13, 14, 15, 16, 17, 76 },
    { 56, 57, 58, 59, 60, 68, 69, 77 },
};

/* GPIOA-E and their reset values (UM GPIO chapter) */
static const uint32_t gpio_addr[] = { 0x40010800, 0x40010C00, 0x40011000,
                                      0x40011400, 0x40011800 };
static const uint32_t gpio_pl_cfg_reset[] = { 0, 0x00080800, 0, 0, 0 };
static const uint32_t gpio_ph_cfg_reset[] = { 0x88800000, 0, 0, 0, 0 };
static const uint32_t gpio_pod_reset[] = { 0xA000, 0x0010, 0, 0, 0 };

/* USART1-3, UART4-7 */
static const uint32_t usart_addr[] = { 0x40013800, 0x40004400, 0x40004800,
                                       0x40004C00, 0x40005000, 0x40015000,
                                       0x40015400 };
static const int usart_irq[] = { 37, 38, 39, 52, 53, 74, 75 };
/* USART1 and UART6/7 are on APB2 */
static const bool usart_apb2[] = { true, false, false, false, false,
                                   true, true };
/* DMA controller (0-based) and request numbers: TX, RX */
static const int usart_dma[][3] = {
    { 0, N32G4FR_DMA1_REQ_USART1_TX, N32G4FR_DMA1_REQ_USART1_RX },
    { 0, N32G4FR_DMA1_REQ_USART2_TX, N32G4FR_DMA1_REQ_USART2_RX },
    { 0, N32G4FR_DMA1_REQ_USART3_TX, N32G4FR_DMA1_REQ_USART3_RX },
    { 1, N32G4FR_DMA2_REQ_UART4_TX, N32G4FR_DMA2_REQ_UART4_RX },
    { 0, N32G4FR_DMA1_REQ_UART5_TX, N32G4FR_DMA1_REQ_UART5_RX },
    { 1, N32G4FR_DMA2_REQ_UART6_TX, N32G4FR_DMA2_REQ_UART6_RX },
    { 1, N32G4FR_DMA2_REQ_UART7_TX, N32G4FR_DMA2_REQ_UART7_RX },
};

/* SPI1-3 */
static const uint32_t spi_addr[] = { 0x40013000, 0x40003800, 0x40003C00 };
static const int spi_irq[] = { 35, 36, 51 };
static const int spi_dma[][3] = {
    { 0, N32G4FR_DMA1_REQ_SPI1_TX, N32G4FR_DMA1_REQ_SPI1_RX },
    { 0, N32G4FR_DMA1_REQ_SPI2_TX, N32G4FR_DMA1_REQ_SPI2_RX },
    { 1, N32G4FR_DMA2_REQ_SPI3_TX, N32G4FR_DMA2_REQ_SPI3_RX },
};

/* ADC1/2 share the ADC1_2 interrupt */
static const uint32_t adc_addr[] = { 0x40020800, 0x40020C00 };
#define ADC_IRQ 18
static const int adc_dma_req[] = { N32G4FR_DMA1_REQ_ADC1,
                                   N32G4FR_DMA1_REQ_ADC2 };

/* TIM1 (update interrupt), TIM2-5 */
static const uint32_t timer_addr[] = { 0x40012C00, 0x40000000, 0x40000400,
                                       0x40000800, 0x40000C00 };
static const int timer_irq[] = { 25, 28, 29, 30, 50 };
static const bool timer_apb2[] = { true, false, false, false, false };

/*
 * Bus clocks as programmed by the vendor SystemInit: SYSCLK = HCLK =
 * 144 MHz, APB2 = HCLK / 2, APB1 = HCLK / 4. Timer clocks are twice their
 * APB clock since the APB prescalers are not 1.
 * TODO: derive these from the RCC once it models clock frequencies.
 */
#define PCLK2_FRQ 72000000
#define PCLK1_FRQ 36000000


static void n32g4fr_soc_initfn(Object *obj)
{
    N32G4FRState *s = N32G4FR_SOC(obj);
    int i;

    object_initialize_child(obj, "armv7m", &s->armv7m, TYPE_ARMV7M);

    object_initialize_child(obj, "rcc", &s->rcc, TYPE_N32G4FR_RCC);
    object_initialize_child(obj, "iwdg", &s->iwdg, TYPE_N32G4FR_IWDG);

    object_initialize_child(obj, "flash", &s->flash, TYPE_N32G4FR_FLASH);

    object_initialize_child(obj, "usbd", &s->usbd, TYPE_N32G4FR_USBD);

    for (i = 0; i < N32_NUM_DMAS; i++) {
        object_initialize_child(obj, "dma[*]", &s->dma[i], TYPE_N32G4FR_DMA);
    }

    for (i = 0; i < N32_NUM_GPIOS; i++) {
        object_initialize_child(obj, "gpio[*]", &s->gpio[i],
                                TYPE_N32G4FR_GPIO);
    }

    for (i = 0; i < N32_NUM_USARTS; i++) {
        object_initialize_child(obj, "usart[*]", &s->usart[i],
                                TYPE_N32G4FR_USART);
    }

    for (i = 0; i < N32_NUM_SPIS; i++) {
        object_initialize_child(obj, "spi[*]", &s->spi[i], TYPE_N32G4FR_SPI);
    }

    object_initialize_child(obj, "adc-orirq", &s->adc_irqs, TYPE_OR_IRQ);
    for (i = 0; i < N32_NUM_ADCS; i++) {
        object_initialize_child(obj, "adc[*]", &s->adc[i], TYPE_N32G4FR_ADC);
    }

    for (i = 0; i < N32_NUM_TIMERS; i++) {
        object_initialize_child(obj, "timer[*]", &s->timer[i],
                                TYPE_STM32F2XX_TIMER);
    }

    s->sysclk = qdev_init_clock_in(DEVICE(s), "sysclk", NULL, NULL, 0);
    s->refclk = qdev_init_clock_in(DEVICE(s), "refclk", NULL, NULL, 0);
}

/* Fill len bytes at buf from a hex string (missing digits read as 0) */
static bool n32g4fr_parse_id(const char *name, const char *hex, uint8_t *buf,
                             size_t len, Error **errp)
{
    size_t i;

    memset(buf, 0, len);
    if (!hex) {
        return true;
    }
    if (strlen(hex) > len * 2) {
        error_setg(errp, "%s must be at most %zu hex digits", name, len * 2);
        return false;
    }
    for (i = 0; hex[i]; i++) {
        int v = g_ascii_xdigit_value(hex[i]);

        if (v < 0) {
            error_setg(errp, "%s: invalid hex digit '%c'", name, hex[i]);
            return false;
        }
        buf[i / 2] |= v << (i % 2 ? 0 : 4);
    }
    return true;
}

static void n32g4fr_soc_realize(DeviceState *dev_soc, Error **errp)
{
    N32G4FRState *s = N32G4FR_SOC(dev_soc);
    MemoryRegion *system_memory = get_system_memory();
    DeviceState *dev, *armv7m;
    SysBusDevice *busdev;
    Error *err = NULL;
    int i;

    /*
     * We use s->refclk internally and only define it with qdev_init_clock_in()
     * so it is correctly parented and not leaked on an init/deinit; it is not
     * intended as an externally exposed clock.
     */
    if (clock_has_source(s->refclk)) {
        error_setg(errp, "refclk clock must not be wired up by the board code");
        return;
    }

    if (!clock_has_source(s->sysclk)) {
        error_setg(errp, "sysclk clock must be wired up by the board code");
        return;
    }

    /*
     * TODO: the RCC only fakes the clock ready/switch status; it does not
     * change the sysclk frequency.
     */

    /* The refclk always runs at frequency HCLK / 8 */
    clock_set_mul_div(s->refclk, 8, 1);
    clock_set_source(s->refclk, s->sysclk);

    /* Flash array and flash interface */
    dev = DEVICE(&s->flash);
    qdev_prop_set_uint32(dev, "flash-size", N32_FLASH_SIZE);
    if (!sysbus_realize(SYS_BUS_DEVICE(dev), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, FLASH_IF_ADDR);
    sysbus_mmio_map(busdev, 1, N32_FLASH_BASE_ADDRESS);
    memory_region_init_alias(&s->flash_alias, OBJECT(dev_soc),
                             "N32G4FR.flash.alias",
                             sysbus_mmio_get_region(busdev, 1), 0,
                             N32_FLASH_SIZE);
    memory_region_add_subregion(system_memory, 0, &s->flash_alias);

    memory_region_init_ram(&s->sram, NULL, "N32G4FR.sram", N32_SRAM_SIZE,
                           &err);
    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }
    memory_region_add_subregion(system_memory, N32_SRAM_BASE_ADDRESS,
                                &s->sram);

    memory_region_init_alias(&s->sram_alias, OBJECT(dev_soc),
                             "N32G4FR.sram.alias", &s->sram, 0,
                             N32_SRAM_ALIAS_SIZE);
    memory_region_add_subregion(system_memory, N32_SRAM_ALIAS_BASE_ADDRESS,
                                &s->sram_alias);

    armv7m = DEVICE(&s->armv7m);
    qdev_prop_set_uint32(armv7m, "num-irq", 96);
    qdev_prop_set_uint8(armv7m, "num-prio-bits", 4);
    qdev_prop_set_string(armv7m, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m4"));
    qdev_prop_set_bit(armv7m, "enable-bitband", true);
    qdev_connect_clock_in(armv7m, "cpuclk", s->sysclk);
    qdev_connect_clock_in(armv7m, "refclk", s->refclk);
    object_property_set_link(OBJECT(&s->armv7m), "memory",
                             OBJECT(system_memory), &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->armv7m), errp)) {
        return;
    }

    sysbus_connect_irq(SYS_BUS_DEVICE(&s->flash), 0,
                       qdev_get_gpio_in(armv7m, FLASH_IRQ));

    /* Reset and clock controller */
    dev = DEVICE(&s->rcc);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->rcc), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, RCC_ADDR);

    /* Independent watchdog: its reset sets RCC_CTRLSTS.IWDGRSTF */
    dev = DEVICE(&s->iwdg);
    if (!sysbus_realize(SYS_BUS_DEVICE(dev), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, IWDG_ADDR);
    qdev_connect_gpio_out_named(dev, "reset-flag", 0,
                                qdev_get_gpio_in_named(DEVICE(&s->rcc),
                                                       "iwdg-reset", 0));

    /* DMA controllers */
    for (i = 0; i < N32_NUM_DMAS; i++) {
        dev = DEVICE(&s->dma[i]);
        qdev_prop_set_uint8(dev, "controller", i + 1);
        object_property_set_link(OBJECT(dev), "downstream",
                                 OBJECT(system_memory), &error_abort);
        if (!sysbus_realize(SYS_BUS_DEVICE(dev), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, dma_addr[i]);
        for (int ch = 0; ch < N32G4FR_DMA_NUM_CHANNELS; ch++) {
            sysbus_connect_irq(busdev, ch,
                               qdev_get_gpio_in(armv7m, dma_irq[i][ch]));
        }
    }

    /* GPIO ports */
    for (i = 0; i < N32_NUM_GPIOS; i++) {
        dev = DEVICE(&s->gpio[i]);
        qdev_prop_set_uint32(dev, "pl-cfg-reset", gpio_pl_cfg_reset[i]);
        qdev_prop_set_uint32(dev, "ph-cfg-reset", gpio_ph_cfg_reset[i]);
        qdev_prop_set_uint32(dev, "pod-reset", gpio_pod_reset[i]);
        if (!sysbus_realize(SYS_BUS_DEVICE(dev), errp)) {
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, gpio_addr[i]);
    }

    /* Attach UART (uses USART registers) and USART controllers */
    for (i = 0; i < N32_NUM_USARTS; i++) {
        DeviceState *dma = DEVICE(&s->dma[usart_dma[i][0]]);

        dev = DEVICE(&(s->usart[i]));
        qdev_prop_set_chr(dev, "chardev", serial_hd(i));
        qdev_prop_set_uint32(dev, "pclk-frequency",
                             usart_apb2[i] ? PCLK2_FRQ : PCLK1_FRQ);
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->usart[i]), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, usart_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, usart_irq[i]));
        qdev_connect_gpio_out_named(dev, N32G4FR_USART_DMA_TX, 0,
                                    qdev_get_gpio_in(dma, usart_dma[i][1]));
        qdev_connect_gpio_out_named(dev, N32G4FR_USART_DMA_RX, 0,
                                    qdev_get_gpio_in(dma, usart_dma[i][2]));
    }

    /*
     * USB device; its virtual COM port goes to the -serial after the
     * USARTs, or -global n32g4fr.usbd.chardev=<id>
     */
    dev = DEVICE(&s->usbd);
    if (serial_hd(N32_NUM_USARTS)) {
        qdev_prop_set_chr(dev, "chardev", serial_hd(N32_NUM_USARTS));
    }
    if (!sysbus_realize(SYS_BUS_DEVICE(dev), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, USBD_ADDR);
    sysbus_mmio_map(busdev, 1, USBD_PMA_ADDR);
    sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, USB_LP_IRQ));
    sysbus_connect_irq(busdev, 1, qdev_get_gpio_in(armv7m, USB_HP_IRQ));

    /* SPI controllers */
    for (i = 0; i < N32_NUM_SPIS; i++) {
        DeviceState *dma = DEVICE(&s->dma[spi_dma[i][0]]);

        dev = DEVICE(&s->spi[i]);
        if (!sysbus_realize(SYS_BUS_DEVICE(dev), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, spi_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, spi_irq[i]));
        qdev_connect_gpio_out_named(dev, N32G4FR_SPI_DMA_TX, 0,
                                    qdev_get_gpio_in(dma, spi_dma[i][1]));
        qdev_connect_gpio_out_named(dev, N32G4FR_SPI_DMA_RX, 0,
                                    qdev_get_gpio_in(dma, spi_dma[i][2]));
    }

    /* ADC devices, the IRQs are ORed together */
    object_property_set_int(OBJECT(&s->adc_irqs), "num-lines", N32_NUM_ADCS,
                            &error_abort);
    if (!qdev_realize(DEVICE(&s->adc_irqs), NULL, errp)) {
        return;
    }
    qdev_connect_gpio_out(DEVICE(&s->adc_irqs), 0,
                          qdev_get_gpio_in(armv7m, ADC_IRQ));

    for (i = 0; i < N32_NUM_ADCS; i++) {
        dev = DEVICE(&s->adc[i]);
        if (!sysbus_realize(SYS_BUS_DEVICE(dev), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, adc_addr[i]);
        sysbus_connect_irq(busdev, 0,
                           qdev_get_gpio_in(DEVICE(&s->adc_irqs), i));
        qdev_connect_gpio_out_named(dev, N32G4FR_ADC_DMA, 0,
                                    qdev_get_gpio_in(DEVICE(&s->dma[0]),
                                                     adc_dma_req[i]));
    }

    /* TIM1 and TIM2 to 5 */
    for (i = 0; i < N32_NUM_TIMERS; i++) {
        dev = DEVICE(&(s->timer[i]));
        qdev_prop_set_uint64(dev, "clock-frequency",
                             2ULL * (timer_apb2[i] ? PCLK2_FRQ : PCLK1_FRQ));
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->timer[i]), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, timer_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, timer_irq[i]));
    }

    /*
     * System configuration area: UCID and UID (UM 31.1), the rest reads
     * erased.  The SIYI firmware reads a 96-bit ID at 0x1FFFF270 first and
     * falls back to the UID only if that reads 0xFFFFFFFF.
     * TODO confirm: 0x1FFFF270 is not documented in UM V3.5.0.
     */
    memory_region_init_rom(&s->syscfg, OBJECT(dev_soc), "N32G4FR.syscfg",
                           N32_SYSCFG_SIZE, &err);
    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }
    {
        uint8_t *p = memory_region_get_ram_ptr(&s->syscfg);

        memset(p, 0xff, N32_SYSCFG_SIZE);
        if (!n32g4fr_parse_id("uid", s->uid, p + N32_UID_OFFSET,
                              N32_UID_SIZE, errp) ||
            !n32g4fr_parse_id("ucid", s->ucid, p + N32_UCID_OFFSET,
                              N32_UCID_SIZE, errp)) {
            return;
        }
    }
    memory_region_add_subregion_overlap(system_memory, N32_SYSCFG_BASE_ADDRESS,
                                        &s->syscfg, 1);

    /* System memory (ROM bootloader) and option bytes */
    create_unimplemented_device("System memory", 0x1FFF0000, 0x10000);

    /* APB1 */
    create_unimplemented_device("timer[6]",    0x40001000, 0x400);
    create_unimplemented_device("timer[7]",    0x40001400, 0x400);
    /*
     * Shown as reserved in the memory map, but the UM USB chapter places
     * the USB DP pull-up enable at bit 28 of 0x40001820.
     */
    create_unimplemented_device("SYSCTL",      0x40001800, 0x400);
    create_unimplemented_device("RTC",         0x40002800, 0x400);
    create_unimplemented_device("WWDG",        0x40002C00, 0x400);
    create_unimplemented_device("I2C1",        0x40005400, 0x400);
    create_unimplemented_device("I2C2",        0x40005800, 0x400);
    create_unimplemented_device("CAN1",        0x40006400, 0x400);
    create_unimplemented_device("CAN2",        0x40006800, 0x400);
    create_unimplemented_device("BKP",         0x40006C00, 0x400);
    create_unimplemented_device("PWR",         0x40007000, 0x400);
    create_unimplemented_device("DAC",         0x40007400, 0x400);

    /* APB2 */
    create_unimplemented_device("AFIO",        0x40010000, 0x400);
    create_unimplemented_device("EXTI",        0x40010400, 0x400);
    create_unimplemented_device("timer[8]",    0x40013400, 0x400);
    create_unimplemented_device("I2C3",        0x40014400, 0x400);
    create_unimplemented_device("I2C4",        0x40014800, 0x400);
    create_unimplemented_device("DVP",         0x40014C00, 0x400);
    create_unimplemented_device("SDIO",        0x40018000, 0x400);

    /* AHB */
    create_unimplemented_device("CRC",         0x40023000, 0x400);
    create_unimplemented_device("SAC",         0x40024000, 0x400);
    create_unimplemented_device("SAC SRAM",    0x40024400, 0x400);
    create_unimplemented_device("MMU",         0x40024C00, 0x400);

    create_unimplemented_device("QSPI bank",   0x90000000, 0x10000000);
    create_unimplemented_device("QSPI",        0xA0001000, 0x400);
}

static const Property n32g4fr_soc_properties[] = {
    DEFINE_PROP_STRING("uid", N32G4FRState, uid),
    DEFINE_PROP_STRING("ucid", N32G4FRState, ucid),
};

static void n32g4fr_soc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = n32g4fr_soc_realize;
    device_class_set_props(dc, n32g4fr_soc_properties);
    /* No vmstate or reset required: device has no internal state */
}

static const TypeInfo n32g4fr_soc_info = {
    .name          = TYPE_N32G4FR_SOC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N32G4FRState),
    .instance_init = n32g4fr_soc_initfn,
    .class_init    = n32g4fr_soc_class_init,
};

static void n32g4fr_soc_types(void)
{
    type_register_static(&n32g4fr_soc_info);
}

type_init(n32g4fr_soc_types)
