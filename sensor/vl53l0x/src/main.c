/**
 * @file vl53.c
 * @brief Minimal I2C driver for VL53-series devices.
 *
 * This driver's only purpose is to probe for a VL53 device on the
 * I2C bus during system startup and confirm it acknowledges its address.
 * It does not implement any sensor-specific data fetching.
 */

#define DT_DRV_COMPAT custom_vl53l0x

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

// Register the module with the Zephyr logging system
LOG_MODULE_REGISTER(VL53_PROBE, CONFIG_SENSOR_LOG_LEVEL);

/*
 * @brief The driver's configuration structure.
 *
 * This is populated at build time by the DEVICE_DT_INST_DEFINE macro
 * and contains a pointer to the I2C device spec from the devicetree.
 */
struct vl53_config {
    struct i2c_dt_spec i2c;
};

/*
 * @brief The driver's data structure.
 *
 * This is for runtime data. For this simple probe driver, it's empty.
 */
struct vl53_data {
    // Empty for this simple probe-only driver
};

/**
 * @brief Probes the I2C bus to check for the VL53 device.
 *
 * This function uses i2c_probe() which sends the I2C address and checks for
 * an ACK (Acknowledge) signal from the peripheral.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @return 0 if the device was found and acknowledged, negative error code otherwise.
 */
static int vl53_check_for_ack(const struct device *dev)
{
    const struct vl53_config *config = dev->config;

    // i2c_probe is the standard Zephyr function to check for an ACK.
    // It sends the address and checks the response from the peripheral.
    // int rc = i2c_probe(&config->i2c);
    int rc = 1;

    if (rc == 0) {
        LOG_INF("VL53 device found at I2C addr 0x%x on bus %s. ACK successful.",
                config->i2c.addr, config->i2c.bus->name);
    } else {
        LOG_ERR("VL53 device not found at I2C addr 0x%x on bus %s. (err %d)",
                config->i2c.addr, config->i2c.bus->name, rc);
    }

    return rc;
}

/**
 * @brief The initialization function for the VL53 driver.
 *
 * This function is called by the Zephyr kernel at boot time.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @return 0 on successful initialization, negative error code otherwise.
 */
static int vl53_init(const struct device *dev)
{
    const struct vl53_config *config = dev->config;

    // Check if the I2C bus controller device is ready.
    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus %s is not ready!", config->i2c.bus->name);
        return -ENODEV;
    }

    // Call our function to perform the ACK check.
    return vl53_check_for_ack(dev);
}

// This section uses the devicetree information to instantiate the driver.
// It creates the necessary config and data structures for each "st,vl53"
// compatible node in the devicetree.

#define VL53_DEFINE(inst)                                           \
    static struct vl53_data vl53_data_##inst;                        \
                                                                    \
    static const struct vl53_config vl53_config_##inst = {           \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                          \
    };                                                              \
                                                                    \
    DEVICE_DT_INST_DEFINE(inst,                                     \
                          &vl53_init,                               \
                          NULL, /* No power management */           \
                          &vl53_data_##inst,                        \
                          &vl53_config_##inst,                      \
                          POST_KERNEL,                              \
                          CONFIG_SENSOR_INIT_PRIORITY,              \
                          NULL); /* No API struct */

DT_INST_FOREACH_STATUS_OKAY(VL53_DEFINE)
