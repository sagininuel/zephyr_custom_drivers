/**
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

 #ifndef _CUSTOM_BNO08X_H
 #define _CUSTOM_BNO08X_H

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "sh2_hal.h"

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>

#include <zephyr/logging/log.h>

typedef uint8_t (*hardwareReset_t) (void);

typedef struct i2c_hal_s {
    sh2_Hal_t sh2_Hal;
    bool isDefault;
    hardwareReset_t reset;
    sh2_EventCallback_t * sh2_event_callback;
} i2c_hal_t;

void bno08x_i2c_hal_init(const struct i2c_dt_spec *i2c_dev, i2c_hal_t * pHal, bool isDefault);





 #endif