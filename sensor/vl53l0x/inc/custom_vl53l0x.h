/**
 * Copyright (c) 2025, Remantek Inc.
 * All rights reserved.
 * 
 * SPDX-License-Identifier: Apache-2.0
 * 
 */


#ifndef ZEPHYR_CUSTOM_VL53L0X_H_
#define ZEPHYR_CUSTOM_VL53L0X_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#include <vl53l0x_def.h>
#include <vl53l0x_platform.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief VL53L0X driver API
 * @note This is redundant in favor of typedef format
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
 * @brief Read distance measurement
 * @param dev VL53L0X device
 * @param distance_mm Distance in millimeters
 * @return 0 on success, negative errno on error
 */
typedef  int (*vl53l0x_read_distance_t)(const struct device *dev, uint16_t *distance_mm);

/**
 * @brief Start continuous measurement
 * @param dev VL53L0X device
 * @return 0 on success, negative errno on error
 */
typedef int (*vl53l0x_start_continuous_t)(const struct device *dev);

/**
 * @brief Stop continuous measurement
 * @param dev VL53L0X device
 * @return 0 on success, negative errno on error
 */
typedef int (*vl53l0x_stop_continuous_t)(const struct device *dev);

/**
 * @brief Check if data is ready
 * @param dev VL53L0X device
 * @param ready Data ready status
 * @return 0 on success, negative errno on error
 */
typedef int (*vl53l0x_is_data_ready_t)(const struct device *dev, bool *ready);

/**
 * @brief Check vl53l0x model ID
 * @param dev VL53L0X device
 * @return 0 on success, negative errno on error
 */
typedef int (*vl53l0x_check_model_id_t)(const struct device *dev);


typedef struct vl53l0x_driver_api_s {
    vl53l0x_read_distance_t read_distance;
    vl53l0x_start_continuous_t start_continuous;
    vl53l0x_stop_continuous_t stop_continuous;
    vl53l0x_is_data_ready_t is_data_ready;
    vl53l0x_check_model_id_t check_model_id;
} vl53l0x_driver_api_t;

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

static inline int vl53l0x_model_id(const struct device * dev)
{
    const vl53l0x_driver_api_t * api = 
        (const vl53l0x_driver_api_t *)dev->api;

    return api->check_model_id(dev);
}

/**
 * @brief Perform ranging test
 */
VL53L0X_Error rangingTest(VL53L0X_Dev_t * pMyDevice);

/**
 * Simple Initialization task
 */
void vl53l0x_task(VL53L0X_Dev_t * );

/**
 * @brief Initialize ble
 */
void ble_init(void);

/**
 * @brief vl53l0x tests utils
 */
void print_pal_error(VL53L0X_Error Status);
void print_range_status(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData);
VL53L0X_Error continuousRangingTest(VL53L0X_Dev_t *pMyDevice);


#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_CUSTOM_VL53L0X_H_ */

