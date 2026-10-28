/*
 * SIYI A8 mini gimbal controller board
 *
 * The gimbal MCU board of the SIYI A8 mini camera: an N32G4FR with, on
 * SPI2, an MPU6509 IMU (chip select PC10) and three magnetic angle
 * encoders (chip selects PB3, PB4, PB5), as used by the vendor firmware.
 * UART5 is the link to the camera's encoder SoC.
 *
 * The vendor application is linked at 0x08003800, behind a bootloader
 * that is not part of the firmware images; the "vtor" machine property
 * (default 0x08003800) provides the vector table address that bootloader
 * would set.
 *
 * The application also reads data the bootloader leaves at 0x08000420
 * onwards (bootloader version and build strings); erased flash there
 * (0xFF) sends it through a bad pointer. Without a flash backing file the
 * bootloader area is therefore zero-filled, standing in for a bootloader
 * that provides nothing. With -drive if=pflash,format=raw,file=... (a
 * 512 KiB flash image, e.g. a dump including the real bootloader) the
 * file's contents are used as they are, and flash writes persist in it.
 *
 * Sensor values can be changed at run time, e.g.
 *   qom-set /machine/imu accel-z 1000
 *   qom-set /machine/encoder[0] angle 16384
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
#include "hw/core/boards.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/loader.h"
#include "hw/core/irq.h"
#include "hw/ssi/ssi.h"
#include "hw/arm/n32g4fr_soc.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "system/blockdev.h"
#include "system/block-backend.h"
#include "hw/core/qdev-properties-system.h"

/* Main SYSCLK frequency in Hz (144MHz) */
#define SYSCLK_FRQ 144000000ULL

/* Vector table of the vendor application, which sits behind a bootloader */
#define APP_VTOR 0x08003800
#define BOOTLOADER_SIZE (APP_VTOR - N32_FLASH_BASE_ADDRESS)

#define NUM_ENCODERS 3
/* Chip selects: GPIO port (0 = A) and pin */
#define IMU_CS_PORT 2
#define IMU_CS_PIN 10
#define ENCODER_CS_PORT 1
static const int encoder_cs_pin[NUM_ENCODERS] = { 3, 4, 5 };

#define TYPE_SIYI_A8_MINI_MACHINE MACHINE_TYPE_NAME("siyi-a8-mini")
OBJECT_DECLARE_SIMPLE_TYPE(SiyiA8MiniMachineState, SIYI_A8_MINI_MACHINE)

struct SiyiA8MiniMachineState {
    MachineState parent_obj;

    uint32_t vtor;
    /* Hex strings for the SoC's UID/UCID */
    char *uid;
    char *ucid;
};

static DeviceState *siyi_a8_mini_spi_device(MachineState *machine,
                                            N32G4FRState *soc,
                                            SSIBus *bus, const char *type,
                                            const char *name, int cs_index,
                                            int port, int pin)
{
    DeviceState *dev = qdev_new(type);

    qdev_prop_set_uint8(dev, "cs", cs_index);
    object_property_add_child(OBJECT(machine), name, OBJECT(dev));
    ssi_realize_and_unref(dev, bus, &error_fatal);
    /* Deselected until the firmware drives the chip select */
    qemu_set_irq(qdev_get_gpio_in_named(dev, SSI_GPIO_CS, 0), 1);
    qdev_connect_gpio_out(DEVICE(&soc->gpio[port]), pin,
                          qdev_get_gpio_in_named(dev, SSI_GPIO_CS, 0));
    return dev;
}

static void siyi_a8_mini_init(MachineState *machine)
{
    SiyiA8MiniMachineState *sms = SIYI_A8_MINI_MACHINE(machine);
    DeviceState *dev;
    N32G4FRState *soc;
    DriveInfo *dinfo;
    SSIBus *spi2;
    Clock *sysclk;

    /* This clock doesn't need migration because it is fixed-frequency */
    sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(sysclk, SYSCLK_FRQ);

    dev = qdev_new(TYPE_N32G4FR_SOC);
    soc = N32G4FR_SOC(dev);
    object_property_add_child(OBJECT(machine), "soc", OBJECT(dev));
    qdev_connect_clock_in(dev, "sysclk", sysclk);
    if (sms->uid) {
        qdev_prop_set_string(dev, "uid", sms->uid);
    }
    if (sms->ucid) {
        qdev_prop_set_string(dev, "ucid", sms->ucid);
    }
    object_property_set_uint(OBJECT(&soc->armv7m), "init-nsvtor", sms->vtor,
                             &error_fatal);
    /* Optional flash backing file: -drive if=pflash,format=raw,file=... */
    dinfo = drive_get(IF_PFLASH, 0, 0);
    if (dinfo) {
        qdev_prop_set_drive_err(DEVICE(&soc->flash), "drive",
                                blk_by_legacy_dinfo(dinfo), &error_fatal);
    } else {
        g_autofree uint8_t *blank = g_malloc0(BOOTLOADER_SIZE);

        rom_add_blob_fixed("siyi-a8-mini.bootloader", blank,
                           BOOTLOADER_SIZE, N32_FLASH_BASE_ADDRESS);
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    spi2 = (SSIBus *)qdev_get_child_bus(DEVICE(&soc->spi[1]), "ssi");
    siyi_a8_mini_spi_device(machine, soc, spi2, "mpu6509", "imu", 0,
                            IMU_CS_PORT, IMU_CS_PIN);
    for (int i = 0; i < NUM_ENCODERS; i++) {
        g_autofree char *name = g_strdup_printf("encoder[%d]", i);

        siyi_a8_mini_spi_device(machine, soc, spi2, "spi-angle-encoder",
                                name, i + 1, ENCODER_CS_PORT,
                                encoder_cs_pin[i]);
    }

    armv7m_load_kernel(soc->armv7m.cpu, machine->kernel_filename,
                       0, N32_FLASH_SIZE);
}

static void siyi_a8_mini_instance_init(Object *obj)
{
    SiyiA8MiniMachineState *sms = SIYI_A8_MINI_MACHINE(obj);

    sms->vtor = APP_VTOR;
    object_property_add_uint32_ptr(obj, "vtor", &sms->vtor,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "vtor",
                                    "Vector table address at reset "
                                    "(default: application base 0x08003800)");
}

static char *siyi_a8_mini_get_uid(Object *obj, Error **errp)
{
    return g_strdup(SIYI_A8_MINI_MACHINE(obj)->uid);
}

static void siyi_a8_mini_set_uid(Object *obj, const char *value, Error **errp)
{
    SiyiA8MiniMachineState *sms = SIYI_A8_MINI_MACHINE(obj);

    g_free(sms->uid);
    sms->uid = g_strdup(value);
}

static char *siyi_a8_mini_get_ucid(Object *obj, Error **errp)
{
    return g_strdup(SIYI_A8_MINI_MACHINE(obj)->ucid);
}

static void siyi_a8_mini_set_ucid(Object *obj, const char *value,
                                  Error **errp)
{
    SiyiA8MiniMachineState *sms = SIYI_A8_MINI_MACHINE(obj);

    g_free(sms->ucid);
    sms->ucid = g_strdup(value);
}

static void siyi_a8_mini_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-m4"),
        NULL
    };

    mc->desc = "SIYI A8 mini gimbal controller (N32G4FR, Cortex-M4)";
    mc->init = siyi_a8_mini_init;
    mc->valid_cpu_types = valid_cpu_types;

    object_class_property_add_str(oc, "uid", siyi_a8_mini_get_uid,
                                  siyi_a8_mini_set_uid);
    object_class_property_set_description(oc, "uid",
                                          "96-bit unique device ID at "
                                          "0x1FFFF7F0, 24 hex digits in "
                                          "address order (default: zeros)");
    object_class_property_add_str(oc, "ucid", siyi_a8_mini_get_ucid,
                                  siyi_a8_mini_set_ucid);
    object_class_property_set_description(oc, "ucid",
                                          "128-bit unique customer ID at "
                                          "0x1FFFF7C0, 32 hex digits in "
                                          "address order (default: zeros)");
}

static const TypeInfo siyi_a8_mini_info = {
    .name          = TYPE_SIYI_A8_MINI_MACHINE,
    .parent        = TYPE_MACHINE,
    .instance_size = sizeof(SiyiA8MiniMachineState),
    .instance_init = siyi_a8_mini_instance_init,
    .class_init    = siyi_a8_mini_class_init,
    .interfaces    = arm_machine_interfaces,
};

static void siyi_a8_mini_register_types(void)
{
    type_register_static(&siyi_a8_mini_info);
}

type_init(siyi_a8_mini_register_types)
