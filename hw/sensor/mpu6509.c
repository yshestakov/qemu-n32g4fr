/*
 * MPU6509 6-axis IMU (SPI), ICM-426xx register compatible
 *
 * The SIYI A8 mini gimbal firmware drives this part through
 * "drv_mpu6509.c": the register map it uses (REG_BANK_SEL 0x76,
 * WHO_AM_I 0x75, DEVICE_CONFIG 0x11, INTF_CONFIG0 0x4C, PWR_MGMT0 0x4E,
 * sensor data 0x1D-0x2A) is that of the TDK ICM-426xx family, but its
 * WHO_AM_I is 0x3D.
 *
 * Model: 4-wire SPI, address byte with bit 7 = read, auto-increment.
 * Sensor data comes from QOM properties in physical units (mg, mdps,
 * milli-degC), scaled by the programmed full scale ranges, so values can
 * be changed at run time with qom-set. Everything else is a plain
 * register file per bank. FIFO, interrupts pins, APEX features and
 * self-test are not modelled.
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
#include "hw/ssi/ssi.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qapi/visitor.h"
#include "qom/object.h"

#define TYPE_MPU6509 "mpu6509"
OBJECT_DECLARE_SIMPLE_TYPE(MPU6509State, MPU6509)

#define NUM_BANKS           5
#define BANK_SIZE           0x80

/* Bank 0 registers */
#define REG_DEVICE_CONFIG   0x11
#define REG_TEMP_DATA1      0x1D
#define REG_GYRO_DATA_Z0    0x2A
#define REG_INT_STATUS      0x2D
#define REG_INTF_CONFIG0    0x4C
#define REG_INTF_CONFIG1    0x4D
#define REG_PWR_MGMT0       0x4E
#define REG_GYRO_CONFIG0    0x4F
#define REG_ACCEL_CONFIG0   0x50
#define REG_WHO_AM_I        0x75
/* All banks */
#define REG_BANK_SEL        0x76

#define DEVICE_CONFIG_SOFT_RESET    BIT(0)
#define INT_STATUS_DATA_RDY         BIT(3)
#define INT_STATUS_RESET_DONE       BIT(4)
#define INTF_CONFIG0_BIG_ENDIAN     BIT(4)

/* Reported when a sensor is off */
#define DATA_INVALID        ((int16_t)0x8000)

struct MPU6509State {
    SSIPeripheral parent_obj;

    uint8_t regs[NUM_BANKS][BANK_SIZE];
    uint8_t bank;
    uint8_t addr;
    bool have_addr;
    bool read;
    bool reset_done;

    /* Sensor inputs */
    int32_t accel_mg[3];
    int32_t gyro_mdps[3];
    int32_t temp_mc;
    uint8_t who_am_i;
};

static void mpu6509_reset_regs(MPU6509State *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0][REG_INTF_CONFIG0] = 0x30;
    s->regs[0][REG_INTF_CONFIG1] = 0x91;
    s->regs[0][REG_GYRO_CONFIG0] = 0x06;
    s->regs[0][REG_ACCEL_CONFIG0] = 0x06;
    s->bank = 0;
}

static int16_t mpu6509_clamp(int64_t v)
{
    return MIN(MAX(v, -32767), 32767);
}

/* Raw 16 bit sample for data word n: 0 temp, 1-3 accel XYZ, 4-6 gyro XYZ */
static int16_t mpu6509_sample(MPU6509State *s, int n)
{
    uint8_t pwr = s->regs[0][REG_PWR_MGMT0];

    if (n == 0) {
        /* Temperature in degC = TEMP_DATA / 132.48 + 25 */
        return mpu6509_clamp((int64_t)(s->temp_mc - 25000) * 13248 / 100000);
    }
    if (n <= 3) {
        /* ACCEL_FS_SEL: 0 = +-16 g (2048 LSB/g) ... 3 = +-2 g */
        int fs = extract32(s->regs[0][REG_ACCEL_CONFIG0], 5, 3) & 3;

        if (!extract32(pwr, 0, 2)) {
            return DATA_INVALID;
        }
        return mpu6509_clamp((int64_t)s->accel_mg[n - 1] * (2048 << fs) /
                             1000);
    }
    /* GYRO_FS_SEL: 0 = +-2000 dps (16.4 LSB/dps), halving per step */
    if (!extract32(pwr, 2, 2)) {
        return DATA_INVALID;
    }
    return mpu6509_clamp((int64_t)s->gyro_mdps[n - 4] *
                         (164 << extract32(s->regs[0][REG_GYRO_CONFIG0],
                                           5, 3)) / 10000);
}

static uint8_t mpu6509_read_reg(MPU6509State *s, uint8_t addr)
{
    uint8_t value;

    if (addr == REG_BANK_SEL) {
        return s->bank;
    }
    if (s->bank != 0) {
        return s->regs[s->bank][addr];
    }

    switch (addr) {
    case REG_TEMP_DATA1 ... REG_GYRO_DATA_Z0: {
        int off = addr - REG_TEMP_DATA1;
        uint16_t sample = mpu6509_sample(s, off / 2);
        bool msb = (off & 1) == 0;

        if (!(s->regs[0][REG_INTF_CONFIG0] & INTF_CONFIG0_BIG_ENDIAN)) {
            msb = !msb;
        }
        value = msb ? sample >> 8 : sample & 0xFF;
        break;
    }
    case REG_INT_STATUS:
        value = INT_STATUS_DATA_RDY |
                (s->reset_done ? INT_STATUS_RESET_DONE : 0);
        s->reset_done = false;
        break;
    case REG_WHO_AM_I:
        value = s->who_am_i;
        break;
    default:
        value = s->regs[0][addr];
        break;
    }
    return value;
}

static void mpu6509_write_reg(MPU6509State *s, uint8_t addr, uint8_t value)
{
    if (addr == REG_BANK_SEL) {
        if (value >= NUM_BANKS) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: bad bank %u\n",
                          __func__, value);
            return;
        }
        s->bank = value;
        return;
    }
    if (s->bank == 0 && addr == REG_DEVICE_CONFIG &&
        (value & DEVICE_CONFIG_SOFT_RESET)) {
        mpu6509_reset_regs(s);
        s->reset_done = true;
        return;
    }
    if (s->bank == 0 && addr == REG_WHO_AM_I) {
        return;
    }
    s->regs[s->bank][addr] = value;
}

static uint32_t mpu6509_transfer(SSIPeripheral *dev, uint32_t val)
{
    MPU6509State *s = MPU6509(dev);
    uint8_t byte = val;
    uint8_t ret = 0;

    if (!s->have_addr) {
        s->read = byte & 0x80;
        s->addr = byte & 0x7F;
        s->have_addr = true;
        return 0;
    }

    if (s->read) {
        ret = mpu6509_read_reg(s, s->addr);
    } else {
        mpu6509_write_reg(s, s->addr, byte);
    }
    trace_mpu6509_access(s->bank, s->addr, s->read, s->read ? ret : byte);
    s->addr = (s->addr + 1) & (BANK_SIZE - 1);
    return ret;
}

static int mpu6509_set_cs(SSIPeripheral *dev, bool level)
{
    MPU6509State *s = MPU6509(dev);

    /* Active low: a new transaction starts with an address byte */
    if (!level) {
        s->have_addr = false;
    }
    return 0;
}

static void mpu6509_reset(DeviceState *dev)
{
    MPU6509State *s = MPU6509(dev);

    mpu6509_reset_regs(s);
    s->have_addr = false;
    s->reset_done = true;
}

static void mpu6509_realize(SSIPeripheral *dev, Error **errp)
{
}

static const VMStateDescription vmstate_mpu6509 = {
    .name = TYPE_MPU6509,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_SSI_PERIPHERAL(parent_obj, MPU6509State),
        VMSTATE_UINT8_2DARRAY(regs, MPU6509State, NUM_BANKS, BANK_SIZE),
        VMSTATE_UINT8(bank, MPU6509State),
        VMSTATE_UINT8(addr, MPU6509State),
        VMSTATE_BOOL(have_addr, MPU6509State),
        VMSTATE_BOOL(read, MPU6509State),
        VMSTATE_BOOL(reset_done, MPU6509State),
        VMSTATE_END_OF_LIST()
    }
};

static const Property mpu6509_properties[] = {
    DEFINE_PROP_UINT8("who-am-i", MPU6509State, who_am_i, 0x3D),
};

static void mpu6509_get_int32(Object *obj, Visitor *v, const char *name,
                              void *opaque, Error **errp)
{
    visit_type_int32(v, name, opaque, errp);
}

static void mpu6509_set_int32(Object *obj, Visitor *v, const char *name,
                              void *opaque, Error **errp)
{
    int32_t value;

    if (visit_type_int32(v, name, &value, errp)) {
        *(int32_t *)opaque = value;
    }
}

static void mpu6509_add_input(Object *obj, const char *name, int32_t *field,
                              int32_t initial, const char *desc)
{
    *field = initial;
    object_property_add(obj, name, "int32", mpu6509_get_int32,
                        mpu6509_set_int32, NULL, field);
    object_property_set_description(obj, name, desc);
}

/* Sensor inputs are plain QOM properties so they stay writable at run time */
static void mpu6509_init(Object *obj)
{
    MPU6509State *s = MPU6509(obj);

    mpu6509_add_input(obj, "accel-x", &s->accel_mg[0], 0, "Acceleration (mg)");
    mpu6509_add_input(obj, "accel-y", &s->accel_mg[1], 0, "Acceleration (mg)");
    mpu6509_add_input(obj, "accel-z", &s->accel_mg[2], 1000,
                      "Acceleration (mg)");
    mpu6509_add_input(obj, "gyro-x", &s->gyro_mdps[0], 0,
                      "Angular rate (mdps)");
    mpu6509_add_input(obj, "gyro-y", &s->gyro_mdps[1], 0,
                      "Angular rate (mdps)");
    mpu6509_add_input(obj, "gyro-z", &s->gyro_mdps[2], 0,
                      "Angular rate (mdps)");
    mpu6509_add_input(obj, "temperature", &s->temp_mc, 25000,
                      "Temperature (milli-degC)");
}

static void mpu6509_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = mpu6509_realize;
    k->transfer = mpu6509_transfer;
    k->set_cs = mpu6509_set_cs;
    k->cs_polarity = SSI_CS_LOW;
    dc->vmsd = &vmstate_mpu6509;
    device_class_set_legacy_reset(dc, mpu6509_reset);
    device_class_set_props(dc, mpu6509_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo mpu6509_info = {
    .name          = TYPE_MPU6509,
    .parent        = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(MPU6509State),
    .instance_init = mpu6509_init,
    .class_init    = mpu6509_class_init,
};

static void mpu6509_register_types(void)
{
    type_register_static(&mpu6509_info);
}

type_init(mpu6509_register_types)
