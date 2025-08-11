// modules/include/drivers/bno08x.h
#ifndef ZEPHYR_INCLUDE_DRIVERS_BNO08X_H_
#define ZEPHYR_INCLUDE_DRIVERS_BNO08X_H_

#include <zephyr/types.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

struct bno08x_data {
    float quat_i;
    float quat_j;
    float quat_k;
    float quat_real;
};

typedef int (*bno08x_read_quaternion_t)(const struct device *dev, struct bno08x_data *data);

__subsystem struct bno08x_driver_api {
    bno08x_read_quaternion_t read_quaternion;
};

static inline int bno08x_read_quaternion(const struct device *dev, struct bno08x_data *data)
{
    const struct bno08x_driver_api *api = 
        (const struct bno08x_driver_api *)dev->api;
    
    return api->read_quaternion(dev, data);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_BNO08X_H_ */

