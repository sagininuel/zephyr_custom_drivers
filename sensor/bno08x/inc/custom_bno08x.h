/**
 * Copyright (c) 2025 Remantek Inc.
 * All Rights Reserved
 * 
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#pragma once

#include <zephyr/types.h>
#include <zephyr/device.h>

#include <bno08x_platform.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct bno08x_quaternion_datas_s {
    float quat_i;
    float quat_j;
    float quat_k;
    float quat_real;
}bno08x_quaternion_data_t;

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

struct bno08x_data {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    const struct custom_driver_api * custom_api;
    sh2_SensorValue_t sensor_value;
    sh2_SensorConfig_t sensor_config;
};

struct bno08x_config {
    struct i2c_dt_spec i2c;
};

typedef struct raw_sensor_data_s{
    sh2_SensorEvent_t * event;
}raw_sensor_data_t;

void print_sh2_SensorEvent_bytes(const sh2_SensorEvent_t *event);

/**
 * @brief Function to call shtp_service() function under hood
 * 
 * @param[in] dev Pointer to device subsystem struct
 * @param[out] sensor_value Pointer to sensor value buffer
 * 
 * @return  
 */
typedef int (*sample_fetch_t)(const struct device *dev, sh2_SensorValue_t *, raw_sensor_data_t *);

/**
 * @brief Function to configure desired reports from bno08x sensor
 * 
 * @param[in] dev Pointer to device subsystem struct
 * @param[in] sensorId ID of streaming sensor data
 * @param[in] interval_us Set streaming interval in microseconds
 * 
 * @return
 */
typedef bool (*bno08x_configure_reports_t)(const struct device *dev, sh2_SensorId_t sensorId, uint32_t interval_us);

__subsystem struct custom_driver_api {
    sample_fetch_t fetch_sample_data;
    bno08x_configure_reports_t configure_reports;
};

static inline bool bno08x_configure_reports(const struct device *dev, sh2_SensorId_t sensorId, uint32_t interval_us)
{
    const struct custom_driver_api *api =
    (const struct custom_driver_api *)dev->api;
    
    return api->configure_reports(dev, sensorId, interval_us);
}

static inline int sample_fetch(const struct device *dev, sh2_SensorValue_t *sensor_value, raw_sensor_data_t * raw_data)
{
    const struct custom_driver_api *api = 
    (const struct custom_driver_api *)dev->api;
    
    return api->fetch_sample_data(dev, sensor_value, raw_data);
}

#ifdef __cplusplus
}
#endif


