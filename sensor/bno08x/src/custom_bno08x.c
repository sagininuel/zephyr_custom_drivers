/**
 * Copyright (c) 2025 Remantek Inc.
 * All rights reserved
 * 
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#define DT_DRV_COMPAT custom_bno08x

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include <custom_bno08x.h>

LOG_MODULE_REGISTER(CUSTOM_BNO08X, LOG_LEVEL_DBG);


i2c_hal_t _bno08x_hal; // Struct representing SH2 Hardware Abstraction Layer
sh2_ProductIds_t prodIds; // Product IDs returned by sensor
sh2_SensorValue_t * _sensor_value = NULL;

// Handle sensor events.
static void sensorHandler(void *cookie, sh2_SensorEvent_t *event) {
  int rc;

//   LOG_DBG("Sensor Handler");
  rc = sh2_decodeSensorEvent(_sensor_value, event);
  if (rc != SH2_OK) {
    LOG_DBG("BNO08x - Error decoding sensor event");
    _sensor_value->timestamp = 0;
    return;
  }
}

static int bno08x_read_reg(const struct device *dev, uint8_t reg, uint8_t *data)
{
    const struct bno08x_config *cfg = dev->config;
    
    return i2c_reg_read_byte_dt(&cfg->i2c, reg, data);
}

// static int bno08x_write_reg(const struct device *dev, uint8_t reg, uint8_t data)
// {
//     const struct bno08x_config *cfg = dev->config;
    
//     return i2c_reg_write_byte_dt(&cfg->i2c, reg, data);
// }

static int bno08x_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct bno08x_data *data = dev->data;
    uint8_t accel_data[6];
    int ret;

    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_ACCEL_XYZ) {
        return -ENOTSUP;
    }

    /* Read accelerometer data - this is simplified for demonstration */
    for (int i = 0; i < 6; i++) {
        ret = bno08x_read_reg(dev, CUSTOM_BNO08X_ACCEL_X_LSB + i, &accel_data[i]);
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

static int bno08x_channel_get(const struct device *dev,
                             enum sensor_channel chan,
                             struct sensor_value *val)
{
    struct bno08x_data *data = dev->data;

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

static int sample_fetch(const struct device * dev, bno08x_quaternion_data_t *data, sh2_SensorValue_t * value)
{
    _sensor_value = value;

    value->timestamp = 0;
    
    sh2_service();

    if(value->timestamp == 0 && value->sensorId != SH2_GYRO_INTEGRATED_RV){
        return false;
    }
    return true;
}

static bool configure_reports_impl (const struct device * dev, sh2_SensorId_t sensorId, uint32_t interval_us)
{
    LOG_DBG("Configure Reports!");
    
    sh2_SensorConfig_t config;

    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.batchInterval_us = 0;
    config.sensorSpecific = 0;

    config.reportInterval_us = interval_us;
    int status = sh2_setSensorConfig(sensorId, &config);

    if (status != SH2_OK) {
        return false;
    }

    LOG_DBG("Configure Reports completed!");
    return true;
}

static const struct sensor_driver_api bno08x_sensor_driver_api = {
    .sample_fetch = bno08x_sample_fetch,
    .channel_get = bno08x_channel_get,
};

static const struct custom_driver_api bno08x_custom_driver_api = {
    .configure_reports = configure_reports_impl,
    .read_quaternion = sample_fetch,
};

static int bno08x_init(const struct device *dev)
{
    const struct bno08x_config *cfg = dev->config;
    struct bno08x_data *data = dev->data;
    data->custom_api = &bno08x_custom_driver_api;

    // uint8_t chip_id;
    int ret;

    LOG_DBG("I2C peripheral BNO08X at address: 0x%02x", cfg->i2c.addr);

    /* Check if I2C device is ready */
    if (!i2c_is_ready_dt(&cfg->i2c)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }
    
    // return 0; // Go to main application (for debug)
    
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

    LOG_DBG("Soft Reset complete..");

    // return 0;

    // Check connection partially by getting the product id's
    memset(&prodIds, 0, sizeof(prodIds));
    ret = sh2_getProdIds(&prodIds);
    if (ret != SH2_OK) {
        LOG_DBG("PRODUCT ID Fail!");
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

    // Register sensor listener
    sh2_setSensorCallback(sensorHandler, NULL);

    LOG_INF("CUSTOM_BNO08X sensor initialized successfully");
    return 0;
}

#define CUSTOM_BNO08X_INIT(inst)                                  \
    static struct bno08x_data bno08x_data_##inst;                 \
                                                                  \
    static const struct bno08x_config bno08x_config_##inst = {    \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                        \
    };                                                            \
                                                                  \
    SENSOR_DEVICE_DT_INST_DEFINE(inst,                            \
                                 bno08x_init,              \
                                 NULL,                            \
                                 &bno08x_data_##inst,             \
                                 &bno08x_config_##inst,           \
                                 POST_KERNEL,                     \
                                 CONFIG_SENSOR_INIT_PRIORITY,     \
                                 &bno08x_sensor_driver_api);      
                                                                  
/*                                                            
    DEVICE_DT_INST_DEFINE(inst,                                   \
                          NULL,                                   \
                          NULL,                                   \
                          &bno08x_data_##inst,                    \
                          &bno08x_config_##inst,                  \
                          POST_KERNEL,                            \
                          CONFIG_SENSOR_INIT_PRIORITY + 1,        \
                          &bno08x_custom_driver_api);             

*/

DT_INST_FOREACH_STATUS_OKAY(CUSTOM_BNO08X_INIT)

