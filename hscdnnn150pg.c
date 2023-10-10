/*
    Honeywell HSCMRRD Pressure Sensor Driver.
    Copyright (C) 2022 AVT, Inc.

    info@avtcare.com
    1125 N 13th St.
    Lafayette, IN 47904

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  US
*/
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/property.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <asm/unaligned.h>

/** linux/units.h not available in kernel v5.4.x, which is our target for this project. */
#define NANO	1000000000UL
#define NOT_READY 0xC0

/*
 * transfer function A: 10% to 90% of 2^14
 * transfer function B: 5% to 95% of 2^14
 * transfer function C: 5% to 85% of 2^14
 * transfer function F: 4% to 94% of 2^14
 */
enum hsc_func_id {
    HSC_FUNCTION_A,
    HSC_FUNCTION_B,
    HSC_FUNCTION_C,
    HSC_FUNCTION_F,
};

struct hsc_func_spec {
    u16             output_min;
    u16             output_max;
};

static const struct hsc_func_spec hsc_func_spec[] = {
    [HSC_FUNCTION_A] = {.output_min = 1638, .output_max = 14745},
    [HSC_FUNCTION_B] = {.output_min = 819, .output_max = 15564},
    [HSC_FUNCTION_C] = {.output_min = 819, .output_max = 13926},
    [HSC_FUNCTION_F] = {.output_min = 655, .output_max = 15400},
};

struct hsc_chan {
    s16     pres;   /* pressure value */
    s16     temp;   /* temperature value */
    s64     ts;     /* timestamp */
};

struct hsc_data {
    struct i2c_client   *client;
    struct mutex        lock;

    u16                 pmin; /* minimal pressure in pascal */
    u16                 pmax;
    enum hsc_func_id  function;
    u16                 outmin; /* minimal numerical range raw value from sensor */
    u16                 outmax; /* maximal numerical range raw value from sensor */
    int                 scale;
    int                 scale2;
    int                 offset;
    int                 offset2;
    struct hsc_chan     chan;
};

static const struct iio_chan_spec hsc_channels[] = {
    {
            .type = IIO_PRESSURE,
            .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
                                    BIT(IIO_CHAN_INFO_SCALE) |
                                    BIT(IIO_CHAN_INFO_OFFSET),
            .scan_index = 0,
            .scan_type = {
                    .sign = 's',
                    .realbits = 14,
                    .storagebits = 16,
                    .endianness = IIO_CPU,
                    // TODO: Shift
            },
        },
    {
            .type = IIO_TEMP,
            .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
            .scan_index = 1,
            .scan_type = {
                .sign = 's',
                .realbits = 11,
                .storagebits = 16,
                .endianness = IIO_CPU,
                // TODO: Shift
            }
        },
        IIO_CHAN_SOFT_TIMESTAMP(1),
};

/**
 * hsc_read_pressure() - Read pressure value from sensor via I2C
 * @data: Pointer to private data struct.
 * @press: Output value read from sensor.
 *
 * Reading from the sensor by sending and receiving I2C telegrams.
 *
 * If there is an end of conversion (EOC) interrupt registered the function
 * waits for a maximum of one second for the interrupt.
 *
 * Context: The function can sleep and data->lock should be held when calling it
 * Return:
 * * 0		- OK, the pressure value could be read
 * * -EIO	- if something goes wrong.
 */
static int hsc_read(struct hsc_data *data, s16 *press, s16 *temp) {
    struct device *dev = &data->client->dev;
    int ret;
    u8 buf[4];

    // Read four bytes into buf.
    ret = i2c_smbus_read_i2c_block_data_or_emulated(data->client, 1, 4, buf);
    if (ret < 0) {
        dev_err(dev, "error in i2c_smbus_read_i2c_block_data ret: %d\n", ret);
        return ret;
    }
    if (ret != sizeof(buf)) {
        dev_err(dev, "received size doesn't fit - ret: %d / %u\n", ret, (u32)sizeof(buf));
        return -EIO;
    }

    // Status bits are bits 7 and 8 of the first byte in the buffer.
    if ((buf[0] & NOT_READY) != 0) {
        dev_err(dev, "Device not ready. ret: %d\n", ret);
        return -EIO;
    }

    *press = get_unaligned_be16(&buf[0]);
    *temp = (get_unaligned_be16(&buf[2]) >> 5);

    return 0;
}

static irqreturn_t hsc_trigger_handler(int irq, void *p) {
    int ret;
    struct iio_poll_func *pf = p;
    struct iio_dev *indio_dev = pf->indio_dev;
    struct hsc_data *data = iio_priv(indio_dev);

    mutex_lock(&data->lock);
    ret = hsc_read(data, &data->chan.pres, &data->chan.temp);

    if (ret >= 0) {
        iio_push_to_buffers_with_timestamp(indio_dev, &data->chan, iio_get_time_ns(indio_dev));
    }

    mutex_unlock(&data->lock);
    iio_trigger_notify_done(indio_dev->trig);

    return IRQ_HANDLED;
}

static int hsc_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val, int *val2, long mask) {
    int ret;
    s16 pressure;
    s16 temp;
    struct hsc_data *data = iio_priv(indio_dev);

    if (chan->type != IIO_PRESSURE) {
        return -EINVAL;
    }

    switch (mask) {
        case IIO_CHAN_INFO_RAW:
            mutex_lock(&data->lock);
            ret = hsc_read(data, &pressure, &temp);
            mutex_unlock(&data->lock);
            if (ret < 0) {
                return ret;
            }

            // Figure out which channel.
            if (chan->scan_index == 0) {
                *val = pressure;
            } else if (chan->scan_index == 1) {
                *val = temp;
            } else {
                return -EINVAL;
            }

            return IIO_VAL_INT;
        case IIO_CHAN_INFO_SCALE:
            if (chan->scan_index == 0) {
                *val = data->scale;
                *val2 = data->scale2;
                return IIO_VAL_INT_PLUS_NANO;
            } else if (chan->scan_index == 1) {
                *val = 0; // TODO: Temp scaling?
                return IIO_VAL_INT;
            }
            return -EINVAL;
        case IIO_CHAN_INFO_OFFSET:
            if (chan->scan_index == 0) {
                *val = data->offset;
                *val2 = data->offset2;
                return IIO_VAL_INT_PLUS_NANO;
            } else if (chan->scan_index == 1) {
                *val = 0; // TODO: Temp offset?
                return IIO_VAL_INT;
            }
            return -EINVAL;
        default:
            return -EINVAL;
    }
}

static const struct iio_info hsc_info = {
        .read_raw = &hsc_read_raw,
};

static int hsc_probe(struct i2c_client *client) {
    int ret;
    struct hsc_data *data;
    struct iio_dev *indio_dev;
    struct device *dev = &client->dev;
    s64 scale, offset;

    indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
    if (!indio_dev) {
        dev_err(dev, "couldn't get iio_dev\n");
        return -ENOMEM;
    }

    data = iio_priv(indio_dev);
    data->client = client;

    mutex_init(&data->lock);

    indio_dev->name = "hscdnnn150pg";
    indio_dev->info = &hsc_info;
    indio_dev->channels = hsc_channels;
    indio_dev->num_channels = ARRAY_SIZE(hsc_channels);
    indio_dev->modes = INDIO_DIRECT_MODE;

    ret = device_property_read_u16(dev, "honeywell,pmin", &data->pmin);
    if (ret) {
        dev_err(dev, "honeywell,pmin could not be read\n");
        return ret;
    }
    ret = device_property_read_u16(dev, "honeywell,pmax", &data->pmax);
    if (ret) {
        dev_err(dev, "honeywell,pmax could not be read\n");
        return ret;
    }
    ret = device_property_read_u32(dev, "honeywell,transfer-function", &data->function);
    if (ret) {
        dev_err(dev, "honeywell,transfer-function could not be read\n");
        return ret;
    }
    if (data->function > HSC_FUNCTION_F) {
        dev_err(dev, "honeywell,transfer-function %d invalid\n", data->function);
        return ret;
    }

    data->outmin = hsc_func_spec[data->function].output_min;
    data->outmax = hsc_func_spec[data->function].output_max;

    /* use 64 bit calculation for preserving a reasonable precision */
    scale = div_s64(((s64)(data->pmax - data->pmin)) * NANO,
                    data->outmax - data->outmin);
    data->scale = div_s64_rem(scale, NANO, &data->scale2);

    // If the min is zero (some gauge devices) we have no offset.
    if (data->pmin == 0) {
        data->offset = 0;
        data->offset2 = 0;
    } else {
        /*
         * multiply with NANO before dividing by scale and later divide by NANO
         * again.
         */
        offset = ((-1LL) * (s64) data->outmin) * NANO -
                 div_s64(div_s64((s64) data->pmin * NANO, scale), NANO);
        data->offset = div_s64_rem(offset, NANO, &data->offset2);
    }

    ret = devm_iio_triggered_buffer_setup(dev, indio_dev, NULL, hsc_trigger_handler, NULL);
    if (ret) {
        dev_err(dev, "unable to register iio device\n");
        return ret;
    }

    return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id hsc_matches[] = {
        { .compatible = "honeywell,hscdnnn150pg"},
        { }
};
MODULE_DEVICE_TABLE(of, hsc_matches);

static const struct i2c_device_id hsc_id[] = {
        { "hscdnnn150pg" },
        { }
};
MODULE_DEVICE_TABLE(i2c, hsc_id);

static struct i2c_driver hsc_driver = {
        .probe_new = hsc_probe,
        .id_table = hsc_id,
        .driver = {
                .name = "hscdnnn150pg",
                .of_match_table = hsc_matches,
        },
};
module_i2c_driver(hsc_driver);

MODULE_AUTHOR("Bryan Varner <bryan.varner@robustified.com>");
MODULE_DESCRIPTION("Honeywell HSCDNNN150PG I2C driver");
MODULE_LICENSE("GPL");