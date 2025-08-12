// zephyr_custom_drivers/modules/include/drivers/vl53l0x.h
#ifndef ZEPHYR_INCLUDE_DRIVERS_VL53L0X_H_
#define ZEPHYR_INCLUDE_DRIVERS_VL53L0X_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief VL53L0X driver API
 */
struct vl53l0x_driver_api {
    /**
     * @brief Read distance measurement
     * @param dev VL53L0X device
     * @param distance_mm Distance in millimeters
     * @return 0 on success, negative errno on error
     */
    int (*read_distance)(const struct device *dev, uint16_t *distance_mm);
    
    /**
     * @brief Start continuous measurement
     * @param dev VL53L0X device
     * @return 0 on success, negative errno on error
     */
    int (*start_continuous)(const struct device *dev);
    
    /**
     * @brief Stop continuous measurement
     * @param dev VL53L0X device
     * @return 0 on success, negative errno on error
     */
    int (*stop_continuous)(const struct device *dev);
    
    /**
     * @brief Check if data is ready
     * @param dev VL53L0X device
     * @param ready Data ready status
     * @return 0 on success, negative errno on error
     */
    int (*is_data_ready)(const struct device *dev, bool *ready);
};

/**
 * @brief Read distance from VL53L0X
 */
static inline int vl53l0x_read_distance(const struct device *dev, uint16_t *distance_mm)
{
    const struct vl53l0x_driver_api *api = 
        (const struct vl53l0x_driver_api *)dev->api;
    
    return api->read_distance(dev, distance_mm);
}

/**
 * @brief Start continuous measurement
 */
static inline int vl53l0x_start_continuous(const struct device *dev)
{
    const struct vl53l0x_driver_api *api = 
        (const struct vl53l0x_driver_api *)dev->api;
    
    return api->start_continuous(dev);
}

/**
 * @brief Stop continuous measurement
 */
static inline int vl53l0x_stop_continuous(const struct device *dev)
{
    const struct vl53l0x_driver_api *api = 
        (const struct vl53l0x_driver_api *)dev->api;
    
    return api->stop_continuous(dev);
}

/**
 * @brief Check if data is ready
 */
static inline int vl53l0x_is_data_ready(const struct device *dev, bool *ready)
{
    const struct vl53l0x_driver_api *api = 
        (const struct vl53l0x_driver_api *)dev->api;
    
    return api->is_data_ready(dev, ready);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_VL53L0X_H_ */

