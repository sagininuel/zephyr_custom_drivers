/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ceva_bno086

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include <bno08x_platform.h>
i2c_hal_t _bno08x_hal; // Struct representing SH2 Hardware Abstraction Layer
sh2_ProductIds_t prodIds; // Product IDs returned by sensor

LOG_MODULE_REGISTER(CUSTOM_BNO08X, CONFIG_SENSOR_LOG_LEVEL);

/* CUSTOM_BNO08X I2C address */
#define CUSTOM_BNO08X_I2C_ADDR_DEFAULT 0x4A

/* CUSTOM_BNO08X Register definitions */
#define CUSTOM_BNO08X_CHIP_ID_REG      0x00
#define CUSTOM_BNO08X_CHIP_ID_VAL      0x86

/* Sample register for demonstration */
#define CUSTOM_BNO08X_ACCEL_X_LSB      0x08
#define CUSTOM_BNO08X_ACCEL_X_MSB      0x09
#define CUSTOM_BNO08X_ACCEL_Y_LSB      0x0A
#define CUSTOM_BNO08X_ACCEL_Y_MSB      0x0B
#define CUSTOM_BNO08X_ACCEL_Z_LSB      0x0C
#define CUSTOM_BNO08X_ACCEL_Z_MSB      0x0D

struct bno086_data {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};

struct bno086_config {
    struct i2c_dt_spec i2c;
};

static int bno086_read_reg(const struct device *dev, uint8_t reg, uint8_t *data)
{
    const struct bno086_config *cfg = dev->config;
    
    return i2c_reg_read_byte_dt(&cfg->i2c, reg, data);
}

static int bno086_write_reg(const struct device *dev, uint8_t reg, uint8_t data)
{
    const struct bno086_config *cfg = dev->config;
    
    return i2c_reg_write_byte_dt(&cfg->i2c, reg, data);
}

static int bno086_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct bno086_data *data = dev->data;
    uint8_t accel_data[6];
    int ret;

    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_ACCEL_XYZ) {
        return -ENOTSUP;
    }

    /* Read accelerometer data - this is simplified for demonstration */
    for (int i = 0; i < 6; i++) {
        ret = bno086_read_reg(dev, CUSTOM_BNO08X_ACCEL_X_LSB + i, &accel_data[i]);
        if (ret < 0) {
            LOG_ERR("Failed to read accelerometer data");
            return ret;
        }
    }

    /* Convert to signed 16-bit values */
    data->accel_x = (int16_t)((accel_data[1] << 8) | accel_data[0]);
    data->accel_y = (int16_t)((accel_data[3] << 8) | accel_data[2]);
    data->accel_z = (int16_t)((accel_data[5] << 8) | accel_data[4]);

    return 0;
}

static int bno086_channel_get(const struct device *dev,
                             enum sensor_channel chan,
                             struct sensor_value *val)
{
    struct bno086_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_ACCEL_X:
        /* Convert raw value to m/s^2 (simplified conversion) */
        val->val1 = data->accel_x / 1000;
        val->val2 = (data->accel_x % 1000) * 1000;
        break;
    case SENSOR_CHAN_ACCEL_Y:
        val->val1 = data->accel_y / 1000;
        val->val2 = (data->accel_y % 1000) * 1000;
        break;
    case SENSOR_CHAN_ACCEL_Z:
        val->val1 = data->accel_z / 1000;
        val->val2 = (data->accel_z % 1000) * 1000;
        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}

static const struct sensor_driver_api bno086_driver_api = {
    .sample_fetch = bno086_sample_fetch,
    .channel_get = bno086_channel_get,
};

static int bno086_init(const struct device *dev)
{
    const struct bno086_config *cfg = dev->config;
    uint8_t chip_id;
    int ret;

    /* Check if I2C device is ready */
    if (!i2c_is_ready_dt(&cfg->i2c)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }
    
    // Get a reference to _bno08x_hal
    i2c_hal_t * bno08x_hal = &_bno08x_hal;

    // Initialize the hal operations
    bno08x_i2c_hal_init(&cfg->i2c, bno08x_hal, true);

    // Reset hardware
    bno08x_hal->reset();

    // Open SH2 interface (also registers non-sensor event handler.)
    ret = sh2_open(&bno08x_hal->sh2_Hal, bno08x_hal->sh2_event_callback, NULL);
    if (ret != SH2_OK){
        return SH2_ERR;
    }

    // Check connection partially by getting the product id's
    memset(&prodIds, 0, sizeof(prodIds));
    ret = sh2_getProdIds(&prodIds);
    if (ret != SH2_OK) {
        return SH2_ERR;
    }
    else{    
        for (int n = 0; n < prodIds.numEntries; n++) {
            printk("Part %d : Version %d.%d.%d Build %d\n",
                prodIds.entry[n].swPartNumber,
                prodIds.entry[n].swVersionMajor, prodIds.entry[n].swVersionMinor,
                prodIds.entry[n].swVersionPatch, prodIds.entry[n].swBuildNumber);
        }
    }


    /* Read and verify chip ID */
    // ret = bno086_read_reg(dev, CUSTOM_BNO08X_CHIP_ID_REG, &chip_id);
    // if (ret < 0) {
    //     LOG_ERR("Failed to read chip ID");
    //     return ret;
    // }

    // if (chip_id != CUSTOM_BNO08X_CHIP_ID_VAL) {
    //     LOG_ERR("Invalid chip ID: 0x%02x (expected 0x%02x)", chip_id, CUSTOM_BNO08X_CHIP_ID_VAL);
    //     return -EINVAL;
    // }

    LOG_INF("CUSTOM_BNO08X sensor initialized successfully");
    return 0;
}

#define CUSTOM_BNO08X_INIT(n)                                          \
    static struct bno086_data bno086_data_##n;                 \
                                                               \
    static const struct bno086_config bno086_config_##n = {   \
        .i2c = I2C_DT_SPEC_INST_GET(n),                       \
    };                                                         \
                                                               \
    SENSOR_DEVICE_DT_INST_DEFINE(n,                           \
                                 bno086_init,                  \
                                 NULL,                         \
                                 &bno086_data_##n,             \
                                 &bno086_config_##n,           \
                                 POST_KERNEL,                  \
                                 CONFIG_SENSOR_INIT_PRIORITY,  \
                                 &bno086_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CUSTOM_BNO08X_INIT)

