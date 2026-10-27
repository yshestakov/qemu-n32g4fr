/*
 * Generic SPI magnetic angle encoder
 *
 * Models the "read angle" exchange common to magnetic rotary encoders
 * such as the MagAlpha MA7xx family: while chip select is low the device
 * shifts out its 16 bit angle MSB first, one byte per 8 bit frame, and
 * ignores the data it receives. Further bytes in the same transaction
 * read as 0.
 *
 * The angle is the "angle" property (0..65535 for one turn) and can be
 * changed at run time with qom-set.
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
#include "trace.h"
#include "hw/ssi/ssi.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_SPI_ANGLE_ENCODER "spi-angle-encoder"
OBJECT_DECLARE_SIMPLE_TYPE(SpiAngleEncoderState, SPI_ANGLE_ENCODER)

struct SpiAngleEncoderState {
    SSIPeripheral parent_obj;

    /* Byte position within the current transaction */
    uint8_t pos;
    uint16_t latched;

    uint16_t angle;
};

static uint32_t spi_angle_encoder_transfer(SSIPeripheral *dev, uint32_t val)
{
    SpiAngleEncoderState *s = SPI_ANGLE_ENCODER(dev);
    uint8_t ret = 0;

    if (s->pos == 0) {
        s->latched = s->angle;
        ret = s->latched >> 8;
    } else if (s->pos == 1) {
        ret = s->latched & 0xFF;
    }
    if (s->pos < UINT8_MAX) {
        s->pos++;
    }
    trace_spi_angle_encoder_transfer(s->latched, ret);
    return ret;
}

static int spi_angle_encoder_set_cs(SSIPeripheral *dev, bool level)
{
    SpiAngleEncoderState *s = SPI_ANGLE_ENCODER(dev);

    /* Active low: restart the angle shift-out */
    if (!level) {
        s->pos = 0;
    }
    return 0;
}

static void spi_angle_encoder_realize(SSIPeripheral *dev, Error **errp)
{
}

static const VMStateDescription vmstate_spi_angle_encoder = {
    .name = TYPE_SPI_ANGLE_ENCODER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_SSI_PERIPHERAL(parent_obj, SpiAngleEncoderState),
        VMSTATE_UINT8(pos, SpiAngleEncoderState),
        VMSTATE_UINT16(latched, SpiAngleEncoderState),
        VMSTATE_END_OF_LIST()
    }
};

/* A plain QOM property so it stays writable at run time */
static void spi_angle_encoder_init(Object *obj)
{
    SpiAngleEncoderState *s = SPI_ANGLE_ENCODER(obj);

    object_property_add_uint16_ptr(obj, "angle", &s->angle,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "angle",
                                    "Angle, 0..65535 for one turn");
}

static void spi_angle_encoder_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = spi_angle_encoder_realize;
    k->transfer = spi_angle_encoder_transfer;
    k->set_cs = spi_angle_encoder_set_cs;
    k->cs_polarity = SSI_CS_LOW;
    dc->vmsd = &vmstate_spi_angle_encoder;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo spi_angle_encoder_info = {
    .name          = TYPE_SPI_ANGLE_ENCODER,
    .parent        = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(SpiAngleEncoderState),
    .instance_init = spi_angle_encoder_init,
    .class_init    = spi_angle_encoder_class_init,
};

static void spi_angle_encoder_register_types(void)
{
    type_register_static(&spi_angle_encoder_info);
}

type_init(spi_angle_encoder_register_types)
