// modules/include/drivers/vl53l0x.h
#ifndef ZEPHYR_INCLUDE_DRIVERS_VL53L0X_H_
#define ZEPHYR_INCLUDE_DRIVERS_VL53L0X_H_

#include <zephyr/types.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*vl53l0x_read_distance_t)(const struct device *dev, uint16_t *distance_mm);

__subsystem struct vl53l0x_driver_api {
    vl53l0x_read_distance_t read_distance;
};

static inline int vl53l0x_read_distance(const struct device *dev, uint16_t *distance_mm)
{
    const struct vl53l0x_driver_api *api = 
        (const struct vl53l0x_driver_api *)dev->api;
    
    return api->read_distance(dev, distance_mm);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_VL53L0X_H_ */

