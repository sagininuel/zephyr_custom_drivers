/*
 * Author: Gini
 * SPDX-License-Identifier: Apache-2.0
 */

// zephyr_custom_drivers/modules/sensor/vl53l0x.c
#define DT_DRV_COMPAT custom_vl53l0x

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include <custom_vl53l0x.h>
#include <vl53l0x_api.h>
#include <vl53l0x_platform.h>
#include <vl53l0x_def.h>

LOG_MODULE_REGISTER(vl53l0x, LOG_LEVEL_DBG);

/* VL53L0X Register Addresses */
/*
#define VL53L0X_REG_SYSRANGE_START                  0x00
#define VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO    0x0A
#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR          0x0B
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0x00C0
#define VL53L0X_REG_RESULT_RANGE_STATUS             0x14
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS         0x13
*/

#define VL53L0X_REG_SYSTEM_FRESH_OUT_OF_RESET 0x016

/* Expected Model ID */
#define VL53L0X_EXPECTED_MODEL_ID 0xEE
#define VL53L0X_EXPECTED_REVISION_ID 0x10

/* Measurement modes */
#define VL53L0X_SINGLE_RANGING_MODE 0x01
#define VL53L0X_CONTINUOUS_RANGING_MODE 0x02

struct vl53l0x_config
{
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec gpio_xshut;
};

struct vl53l0x_data
{
    bool continuous_mode;
    VL53L0X_Dev_t device;
    uint16_t last_distance;
};

static int vl53l0x_reg_read_u8(const struct device *dev, uint8_t reg, uint8_t *val)
{
    const struct vl53l0x_config *config = dev->config;

    return i2c_reg_read_byte_dt(&config->i2c, reg, val);
}

static int vl53l0x_reg_write_u8(const struct device *dev, uint8_t reg, uint8_t val)
{
    const struct vl53l0x_config *config = dev->config;

    return i2c_reg_write_byte_dt(&config->i2c, reg, val);
}

static int vl53l0x_reg_read_u16(const struct device *dev, uint8_t reg, uint16_t *val)
{
    const struct vl53l0x_config *config = dev->config;
    uint8_t buf[2];
    int ret;

    ret = i2c_burst_read_dt(&config->i2c, reg, buf, sizeof(buf));
    if (ret < 0)
    {
        return ret;
    }

    *val = sys_get_be16(buf);
    return 0;
}

static int vl53l0x_check_model_id(const struct device *dev)
{
    uint8_t model_id;
    uint8_t revision_id;
    uint16_t timeout_default;
    int ret;

    ret = vl53l0x_reg_read_u8(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model_id);
    ret = vl53l0x_reg_read_u8(dev, VL53L0X_REG_IDENTIFICATION_REVISION_ID, &revision_id);
    ret = vl53l0x_reg_read_u16(dev, VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &timeout_default);
    if (ret < 0)
    {
        LOG_ERR("Failed to read model ID: %d", ret);
        return ret;
    }

    if (model_id != VL53L0X_EXPECTED_MODEL_ID)
    {
        LOG_ERR("Unexpected model ID: 0x%02x (expected 0x%02x)",
                model_id, VL53L0X_EXPECTED_MODEL_ID);
        return -ENODEV;
    }

    if (revision_id != VL53L0X_EXPECTED_REVISION_ID)
    {
        LOG_ERR("Unexpected revision ID :0x%02x (expected 0x%02x)", 
                revision_id, VL53L0X_EXPECTED_REVISION_ID);
        return -ENODEV;
    }

    LOG_INF("Model ID verified: 0x%02x", model_id);
    LOG_INF("Revision ID verified: 0x%02x", revision_id);
    return 0;
}

static struct vl53l0x_data * driver_data;

static int vl53l0x_init_sensor(const struct device *dev)
{
    int ret = 0;
    // uint8_t val;

    // VL53L0X_Dev_t MyDevice;
    // VL53L0X_Dev_t * pMyDevice = &MyDevice;
    VL53L0X_DeviceInfo_t DeviceInfo;

    driver_data = dev->data;
    driver_data->device.dev = dev;

    // VL53L0X_Dev_t vl53l0x_device = { 0 };
    // VL53L0X_Dev_t * device = &vl53l0x_device;
    // device->dev = dev;

    if(ret == VL53L0X_ERROR_NONE)
    {
        ret = VL53L0X_GetDeviceInfo(&driver_data->device, &DeviceInfo);
        // ret = -1;
        if(ret == VL53L0X_ERROR_NONE)
        {
            LOG_DBG("VL53L0X_GetDeviceInfo:\n");
            LOG_DBG("Device Name : %s\n", DeviceInfo.Name);
            LOG_DBG("Device Type : %s\n", DeviceInfo.Type);
            LOG_DBG("Device ID : %s\n", DeviceInfo.ProductId);
            LOG_DBG("ProductRevisionMajor : %d\n", DeviceInfo.ProductRevisionMajor);
            LOG_DBG("ProductRevisionMinor : %d\n", DeviceInfo.ProductRevisionMinor);

        if ((DeviceInfo.ProductRevisionMinor != 1) && (DeviceInfo.ProductRevisionMinor != 1)) {
                LOG_ERR("Error expected cut 1.1 but found cut %d.%d\n",
                       DeviceInfo.ProductRevisionMajor, DeviceInfo.ProductRevisionMinor);
                ret = VL53L0X_ERROR_NOT_SUPPORTED;
            }
        }
        LOG_DBG("Status: %d", ret);
    }

    // Data Init
    LOG_DBG("Before init .. %d", driver_data->device.Data.DeviceSpecificParameters.ModuleId);
    ret = VL53L0X_DataInit(&driver_data->device);
    if (ret < 0)
    {
        LOG_ERR("ERROR at init..");
        return ret;
    }

    if(ret == VL53L0X_ERROR_NONE)
    {
        LOG_DBG("Ready for a ranging test ..");
        ret = rangingTest(&driver_data->device);
        // vl53l0x_task(&driver_data->device);
    }

    LOG_DBG("VL53L0X sensor initialized successfully");
    return 0;
}


static int vl53l0x_read_distance_impl(const struct device * dev, uint16_t * distance_mm)
{   
    VL53L0X_Error status =  VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t RangingMeasurementData;

    // struct vl53l0x_data *driver_data = dev->data;
    // driver_data->device.dev = dev;

    LOG_DBG("Read distance ..");
    status = VL53L0X_PerformSingleRangingMeasurement(&driver_data->device, &RangingMeasurementData);
    print_pal_error(status);
    print_range_status(&RangingMeasurementData);
    LOG_DBG("Return Status: %d\n", status);

    LOG_DBG("Measured distance: %i\n\n", RangingMeasurementData.RangeMilliMeter);
    *distance_mm =  RangingMeasurementData.RangeMilliMeter;

    if (status == 0)
    {
        LOG_DBG("Measured distance: %i\n\n", RangingMeasurementData.RangeMilliMeter);
    }

    return status;
}

static int vl53l0x_start_continuous_impl(const struct device *dev)
{
    // struct vl53l0x_data *data = dev->data;
    // int ret;

    // if (data->continuous_mode)
    // {
    //     return 0; /* Already in continuous mode */
    // }

    // ret = vl53l0x_reg_write_u8(dev, VL53L0X_REG_SYSRANGE_START, VL53L0X_CONTINUOUS_RANGING_MODE);
    // if (ret < 0)
    // {
    //     LOG_ERR("Failed to start continuous mode: %d", ret);
    //     return ret;
    // }

    // data->continuous_mode = true;
    LOG_DBG("Continuous mode started");
    continuousRangingTest(&driver_data->device);
    return 0;
}

static int vl53l0x_stop_continuous_impl(const struct device *dev)
{
    struct vl53l0x_data *data = dev->data;
    int ret;

    if (!data->continuous_mode)
    {
        return 0; /* Already stopped */
    }

    ret = vl53l0x_reg_write_u8(dev, VL53L0X_REG_SYSRANGE_START, 0x01); /* Stop bit */
    if (ret < 0)
    {
        LOG_ERR("Failed to stop continuous mode: %d", ret);
        return ret;
    }

    data->continuous_mode = false;
    LOG_DBG("Continuous mode stopped");
    return 0;
}

static int vl53l0x_is_data_ready_impl(const struct device *dev, bool *ready)
{
    uint8_t status;
    int ret;

    ret = vl53l0x_reg_read_u8(dev, VL53L0X_REG_RESULT_INTERRUPT_STATUS, &status);
    if (ret < 0)
    {
        return ret;
    }

    *ready = (status & 0x07) != 0;
    return 0;
}

static int vl53l0x_init(const struct device *dev)
{
    const struct vl53l0x_config *config = dev->config;
    struct vl53l0x_data *data = dev->data;
    int ret;

    LOG_INF("I2C peripheral 'VL53L0x' at address: 0x%02x", config->i2c.addr);

    /* Check I2C bus */
    if (!i2c_is_ready_dt(&config->i2c))
    {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    /* Initialize XSHUT GPIO if available */
    if (config->gpio_xshut.port)
    {
        if (!gpio_is_ready_dt(&config->gpio_xshut))
        {
            LOG_ERR("XSHUT GPIO not ready");
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&config->gpio_xshut, GPIO_OUTPUT_ACTIVE);
        if (ret < 0)
        {
            LOG_ERR("Failed to configure XSHUT GPIO: %d", ret);
            return ret;
        }

        /* Ensure sensor is not in shutdown */
        gpio_pin_set_dt(&config->gpio_xshut, 1);
        k_msleep(10); /* Wait for sensor to start up */
    }

    /* Verify sensor presence */
    ret = vl53l0x_check_model_id(dev);
    if (ret < 0)
    {
        return ret;
    }

    /* Initialize sensor */
    ret = vl53l0x_init_sensor(dev);
    if (ret < 0)
    {
        return ret;
    }

    /* Initialize driver data */
    data->continuous_mode = false;
    data->last_distance = 0;

    LOG_INF("VL53L0X initialized successfully");
    return 0;
}

static const vl53l0x_driver_api_t vl53l0x_api = {
    .read_distance = vl53l0x_read_distance_impl,
    .start_continuous = vl53l0x_start_continuous_impl,
    .stop_continuous = vl53l0x_stop_continuous_impl,
    .is_data_ready = vl53l0x_is_data_ready_impl,
    .check_model_id = vl53l0x_check_model_id,
};


#define VL53L0X_DEFINE(inst)                                            \
    static struct vl53l0x_data vl53l0x_data_##inst;                     \
                                                                        \
    static const struct vl53l0x_config vl53l0x_config_##inst = {        \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                              \
        .gpio_xshut = GPIO_DT_SPEC_INST_GET_OR(inst, xshut_gpios, {0}), \
    };                                                                  \
                                                                        \
    DEVICE_DT_INST_DEFINE(inst,                                         \
                          vl53l0x_init,                                 \
                          NULL,                                         \
                          &vl53l0x_data_##inst,                         \
                          &vl53l0x_config_##inst,                       \
                          POST_KERNEL,                                  \
                          CONFIG_SENSOR_INIT_PRIORITY,                  \
                          &vl53l0x_api);

DT_INST_FOREACH_STATUS_OKAY(VL53L0X_DEFINE)
