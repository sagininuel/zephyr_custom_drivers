/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_vl53l0x

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(vl53l0x, CONFIG_SENSOR_LOG_LEVEL);

/* VL53L0X Register Map */
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0xC0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID      0xC2
#define VL53L0X_REG_SYSRANGE_START                  0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS         0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS             0x14
#define VL53L0X_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN   0xBC
#define VL53L0X_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN    0xC0
#define VL53L0X_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF   0xD0
#define VL53L0X_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_REF    0xD4
#define VL53L0X_REG_RESULT_PEAK_SIGNAL_RATE_REF             0xB6

/* Expected chip ID */
#define VL53L0X_CHIP_ID                             0xEE

/* Commands */
#define VL53L0X_SYSRANGE_MODE_START_STOP            0x01
#define VL53L0X_SYSRANGE_MODE_SINGLESHOT            0x00
#define VL53L0X_SYSRANGE_MODE_BACKTOBACK            0x02
#define VL53L0X_SYSRANGE_MODE_TIMED                 0x04
#define VL53L0X_SYSRANGE_MODE_HISTOGRAM             0x08

struct vl53l0x_data {
    uint16_t distance;
    uint16_t ambient_rate;
    uint16_t signal_rate;
    uint8_t range_status;
    
#ifdef CONFIG_VL53L0X_INTERRUPT
    struct gpio_callback int_callback;
    const struct device *dev;
    sensor_trigger_handler_t data_ready_handler;
    const struct sensor_trigger *data_ready_trigger;
#endif
};

struct vl53l0x_config {
    struct i2c_dt_spec i2c;
#ifdef CONFIG_VL53L0X_INTERRUPT
    struct gpio_dt_spec int_gpio;
#endif
    struct gpio_dt_spec xshut_gpio;
};

static int vl53l0x_reg_read(const struct device *dev, uint8_t reg, uint8_t *data)
{
    const struct vl53l0x_config *config = dev->config;
    
    return i2c_reg_read_byte_dt(&config->i2c, reg, data);
}

static int vl53l0x_reg_write(const struct device *dev, uint8_t reg, uint8_t data)
{
    const struct vl53l0x_config *config = dev->config;
    
    return i2c_reg_write_byte_dt(&config->i2c, reg, data);
}

static int vl53l0x_reg_read_multi(const struct device *dev, uint8_t reg, 
                                  uint8_t *data, uint8_t len)
{
    const struct vl53l0x_config *config = dev->config;
    
    return i2c_burst_read_dt(&config->i2c, reg, data, len);
}

static int vl53l0x_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct vl53l0x_data *data = dev->data;
    uint8_t status;
    uint8_t range_data[12];
    int ret;
    
    ARG_UNUSED(chan);
    
    /* Start ranging */
    ret = vl53l0x_reg_write(dev, VL53L0X_REG_SYSRANGE_START, 
                           VL53L0X_SYSRANGE_MODE_START_STOP);
    if (ret < 0) {
        LOG_ERR("Failed to start ranging");
        return ret;
    }
    
    /* Wait for measurement completion */
    int timeout = CONFIG_VL53L0X_MAX_CONVERGENCE_TIME;
    do {
        k_usleep(1000);
        ret = vl53l0x_reg_read(dev, VL53L0X_REG_RESULT_RANGE_STATUS, &status);
        if (ret < 0) {
            LOG_ERR("Failed to read status");
            return ret;
        }
        timeout -= 1000;
    } while (!(status & 0x01) && timeout > 0);
    
    if (timeout <= 0) {
        LOG_ERR("Timeout waiting for measurement");
        return -EIO;
    }
    
    /* Read measurement data */
    ret = vl53l0x_reg_read_multi(dev, VL53L0X_REG_RESULT_RANGE_STATUS, 
                                range_data, sizeof(range_data));
    if (ret < 0) {
        LOG_ERR("Failed to read measurement data");
        return ret;
    }
    
    data->range_status = range_data[0];
    data->distance = sys_get_be16(&range_data[10]);
    data->signal_rate = sys_get_be16(&range_data[6]);
    data->ambient_rate = sys_get_be16(&range_data[8]);
    
    return 0;
}

static int vl53l0x_channel_get(const struct device *dev, enum sensor_channel chan,
                              struct sensor_value *val)
{
    struct vl53l0x_data *data = dev->data;
    
    switch (chan) {
    case SENSOR_CHAN_DISTANCE:
        /* Distance in millimeters, convert to meters */
        val->val1 = data->distance / 1000;
        val->val2 = (data->distance % 1000) * 1000;
        break;
        
    case SENSOR_CHAN_PROX:
        /* Raw distance in mm */
        val->val1 = data->distance;
        val->val2 = 0;
        break;
        
    case SENSOR_CHAN_LIGHT:
        /* Ambient light rate */
        val->val1 = data->ambient_rate;
        val->val2 = 0;
        break;
        
    default:
        return -ENOTSUP;
    }
    
    return 0;
}

#ifdef CONFIG_VL53L0X_INTERRUPT
static void vl53l0x_int_callback(const struct device *dev,
                                struct gpio_callback *cb, uint32_t pins)
{
    struct vl53l0x_data *data = CONTAINER_OF(cb, struct vl53l0x_data, int_callback);
    
    ARG_UNUSED(pins);
    
    if (data->data_ready_handler != NULL) {
        data->data_ready_handler(data->dev, data->data_ready_trigger);
    }
}

static int vl53l0x_trigger_set(const struct device *dev,
                              const struct sensor_trigger *trig,
                              sensor_trigger_handler_t handler)
{
    const struct vl53l0x_config *config = dev->config;
    struct vl53l0x_data *data = dev->data;
    
    if (!config->int_gpio.port) {
        return -ENOTSUP;
    }
    
    if (trig->type != SENSOR_TRIG_DATA_READY) {
        return -ENOTSUP;
    }
    
    data->data_ready_handler = handler;
    data->data_ready_trigger = trig;
    
    return 0;
}
#endif /* CONFIG_VL53L0X_INTERRUPT */

static int vl53l0x_init(const struct device *dev)
{
    const struct vl53l0x_config *config = dev->config;
    struct vl53l0x_data *data = dev->data;
    uint8_t chip_id;
    int ret;
    
    if (!i2c_is_ready_dt(&config->i2c)) {
        LOG_ERR("I2C bus is not ready");
        return -ENODEV;
    }
    
    /* Configure XSHUT pin if available */
    if (config->xshut_gpio.port) {
        ret = gpio_pin_configure_dt(&config->xshut_gpio, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure XSHUT pin");
            return ret;
        }
        
        /* Reset sequence */
        gpio_pin_set_dt(&config->xshut_gpio, 0);
        k_msleep(10);
        gpio_pin_set_dt(&config->xshut_gpio, 1);
        k_msleep(10);
    }
    
    /* Read chip ID */
    ret = vl53l0x_reg_read(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &chip_id);
    if (ret < 0) {
        LOG_ERR("Failed to read chip ID");
        return ret;
    }
    
    if (chip_id != VL53L0X_CHIP_ID) {
        LOG_ERR("Invalid chip ID: 0x%02X (expected 0x%02X)", chip_id, VL53L0X_CHIP_ID);
        return -ENODEV;
    }
    
    LOG_INF("VL53L0X chip ID: 0x%02X", chip_id);
    
#ifdef CONFIG_VL53L0X_INTERRUPT
    /* Configure interrupt pin */
    if (config->int_gpio.port) {
        ret = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure interrupt pin");
            return ret;
        }
        
        gpio_init_callback(&data->int_callback, vl53l0x_int_callback,
                          BIT(config->int_gpio.pin));
        
        ret = gpio_add_callback(config->int_gpio.port, &data->int_callback);
        if (ret < 0) {
            LOG_ERR("Failed to set interrupt callback");
            return ret;
        }
        
        data->dev = dev;
    }
#endif
    
    return 0;
}

static const struct sensor_driver_api vl53l0x_driver_api = {
    .sample_fetch = vl53l0x_sample_fetch,
    .channel_get = vl53l0x_channel_get,
#ifdef CONFIG_VL53L0X_INTERRUPT
    .trigger_set = vl53l0x_trigger_set,
#endif
};

#define VL53L0X_DEFINE(inst)                                                   \
    static struct vl53l0x_data vl53l0x_data_##inst;                          \
                                                                              \
    static const struct vl53l0x_config vl53l0x_config_##inst = {             \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                   \
        IF_ENABLED(CONFIG_VL53L0X_INTERRUPT,                                  \
                  (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),)) \
        .xshut_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, xshut_gpios, {0}),      \
    };                                                                        \
                                                                              \
    SENSOR_DEVICE_DT_INST_DEFINE(inst,                                        \
                                 vl53l0x_init,                               \
                                 NULL,                                        \
                                 &vl53l0x_data_##inst,                       \
                                 &vl53l0x_config_##inst,                     \
                                 POST_KERNEL,                                 \
                                 CONFIG_VL53L0X_INIT_PRIORITY,               \
                                 &vl53l0x_driver_api);

DT_INST_FOREACH_STATUS_OKAY(VL53L0X_DEFINE)

