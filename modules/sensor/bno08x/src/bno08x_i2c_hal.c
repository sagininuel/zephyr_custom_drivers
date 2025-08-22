/**
 * SPDX-License-Identifier: Apache-2.0
 * 
 */


 //Implementation of the BNO08X i2c Hardware Abstraction Layer
#include <bno08x_platform.h>

// static const struct device *i2c_device;
static bool _reset_occurred = false;

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
// #include <zephyr/sys/time_units.h>

LOG_MODULE_REGISTER(i2c_hal, LOG_LEVEL_DBG);

// I2C device tree specification - should be defined based on your device tree
// static const struct i2c_dt_spec i2c_spec = I2C_DT_SPEC_INST_GET(0);
static const struct i2c_dt_spec * pHal_i2c;

// Configuration structure for buffer management
typedef struct i2c_hal_config_s {
    size_t max_buffer_size_in;
    size_t max_buffer_size_out;
} i2c_hal_config_t;

static i2c_hal_config_t i2c_hal_cfg = {
    .max_buffer_size_in = 32, // Default I2C buffer size
    .max_buffer_size_out = SH2_HAL_MAX_PAYLOAD_OUT
};

static int i2chal_open(sh2_Hal_t *self) {
    LOG_DBG("I2C HAL open");
    
    // Check if I2C device is ready
    if (!i2c_is_ready_dt(pHal_i2c)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }
    
    uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
    bool success = false;
    int ret;
    
    for (uint8_t attempts = 0; attempts < 5; attempts++) {
        ret = i2c_burst_write_dt(pHal_i2c, 0x00, softreset_pkt, sizeof(softreset_pkt));
        if (ret == 0) {
            success = true;
            break;
        }
        LOG_WRN("I2C write attempt %d failed: %d", attempts + 1, ret);
        k_msleep(30);
    }
    
    if (!success) {
        LOG_ERR("Failed to send soft reset after 5 attempts");
        return -EIO;
    }
    
    k_msleep(300);
    return 0;
}

static void i2chal_close(sh2_Hal_t *self) {
    LOG_DBG("I2C HAL close");
    // In Zephyr, typically no explicit close needed for I2C
}

static int i2chal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                       uint32_t *t_us) {
    LOG_DBG("I2C HAL read");
    
    if (!i2c_is_ready_dt(pHal_i2c)) {
        LOG_ERR("I2C device not ready");
        return 0;
    }
    
    uint8_t header[4];
    int ret = i2c_burst_read_dt(pHal_i2c, 0x00, header, sizeof(header));
    if (ret != 0) {
        LOG_ERR("Failed to read I2C header: %d", ret);
        return 0;
    }
    
    // Determine amount to read
    uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
    // Unset the "continue" bit
    packet_size &= ~0x8000;
    
    LOG_DBG("Packet size: %u, buffer size: %u", packet_size, len);
    
    size_t i2c_buffer_max = i2c_hal_cfg.max_buffer_size_in;
    
    if (packet_size > len) {
        LOG_WRN("Packet size %u exceeds buffer size %u", packet_size, len);
        return 0;
    }
    
    // the number of non-header bytes to read
    uint16_t cargo_remaining = packet_size;
    uint8_t i2c_buffer[i2c_buffer_max];
    uint16_t read_size;
    uint16_t cargo_read_amount = 0;
    bool first_read = true;
    
    while (cargo_remaining > 0) {
        if (first_read) {
            read_size = MIN(i2c_buffer_max, (size_t)cargo_remaining);
        } else {
            read_size = MIN(i2c_buffer_max, (size_t)cargo_remaining + 4);
        }
        
        LOG_DBG("Reading from I2C: %u bytes, remaining: %u", 
                read_size, cargo_remaining);
        
        ret = i2c_burst_read_dt(pHal_i2c, 0x00, i2c_buffer, read_size);
        if (ret != 0) {
            LOG_ERR("I2C read failed: %d", ret);
            return 0;
        }
        
        if (first_read) {
            // The first time we're saving the "original" header, so include it in the
            // cargo count
            cargo_read_amount = read_size;
            memcpy(pBuffer, i2c_buffer, cargo_read_amount);
            first_read = false;
        } else {
            // this is not the first read, so copy from 4 bytes after the beginning of
            // the i2c buffer to skip the header included with every new i2c read and
            // don't include the header in the amount of cargo read
            cargo_read_amount = read_size - 4;
            memcpy(pBuffer, i2c_buffer + 4, cargo_read_amount);
        }
        // advance our pointer by the amount of cargo read
        pBuffer += cargo_read_amount;
        // mark the cargo as received
        cargo_remaining -= cargo_read_amount;
    }
    
    // Set timestamp if requested
    // if (t_us) {
    //     *t_us = k_cyc_to_us_ceil32(k_cycle_get_32());
    // }
    
    return packet_size;
}

static int i2chal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    if (!i2c_is_ready_dt(pHal_i2c)) {
        LOG_ERR("I2C device not ready");
        return 0;
    }
    
    // size_t i2c_buffer_max = i2c_hal_cfg.max_buffer_size_in;
    
    // LOG_DBG("I2C HAL write packet size: %u, max buffer size: %zu", 
    //         len, i2c_buffer_max);
    
    // uint16_t write_size = MIN(i2c_buffer_max, len);
    int ret = i2c_burst_write_dt(pHal_i2c, 0x00, pBuffer, len);
    if (ret != 0) {
        LOG_ERR("I2C write failed: %d", ret);
        return 0;
    }
    
    return len;
}

// Initialization function to set up I2C configuration
int i2c_hal_init(size_t max_buffer_size_in) {
    if (max_buffer_size_in > 0) {
        i2c_hal_cfg.max_buffer_size_in = max_buffer_size_in;
    }
    
    // Check if I2C device is ready
    if (!i2c_is_ready_dt(pHal_i2c)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }
    
    LOG_INF("I2C HAL initialized successfully");
    LOG_INF("I2C device: %s, address: 0x%02X", 
            pHal_i2c->bus->name, pHal_i2c->addr);
    return 0;
}

// Alternative functions for different register addresses if needed
static int i2chal_read_reg(sh2_Hal_t *self, uint8_t reg_addr, 
                          uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    LOG_DBG("I2C HAL read from register 0x%02X", reg_addr);
    
    if (!i2c_is_ready_dt(pHal_i2c)) {
        LOG_ERR("I2C device not ready");
        return 0;
    }
    
    int ret = i2c_burst_read_dt(pHal_i2c, reg_addr, pBuffer, len);
    if (ret != 0) {
        LOG_ERR("Failed to read from register 0x%02X: %d", reg_addr, ret);
        return 0;
    }
    
    // Set timestamp if requested
    // if (t_us) {
    //     *t_us = k_cyc_to_us_ceil32(k_cycle_get_32());
    // }
    
    return len;
}

static int i2chal_write_reg(sh2_Hal_t *self, uint8_t reg_addr, 
                           uint8_t *pBuffer, unsigned len) {
    LOG_DBG("I2C HAL write to register 0x%02X", reg_addr);
    
    if (!i2c_is_ready_dt(pHal_i2c)) {
        LOG_ERR("I2C device not ready");
        return 0;
    }
    
    int ret = i2c_burst_write_dt(pHal_i2c, reg_addr, pBuffer, len);
    if (ret != 0) {
        LOG_ERR("Failed to write to register 0x%02X: %d", reg_addr, ret);
        return 0;
    }
    
    return len;
}



static uint8_t hardware_reset(void)
{
    return 0;
}

static void event_callback(void *cookie, sh2_AsyncEvent_t * pEvent)
{
 // If we see a reset, set a flag so that sensors will be reconfigured.
  if (pEvent->eventId == SH2_RESET) {
    _reset_occurred = true;
  }
}

void bno08x_i2c_hal_init(const struct i2c_dt_spec * i2c_dev, i2c_hal_t * pHal, bool defaultState)
{
    pHal_i2c = i2c_dev;
    pHal->isDefault = defaultState;
    pHal->reset = hardware_reset;
    pHal->sh2_event_callback = event_callback;

    pHal->sh2_Hal.open = i2chal_open;
    pHal->sh2_Hal.close = i2chal_close;
    pHal->sh2_Hal.read = i2chal_read;
    pHal->sh2_Hal.write = i2chal_write;
    pHal->sh2_Hal.getTimeUs = NULL;

    // Check i2c peripheral is ready.
    i2c_hal_init(SH2_HAL_MAX_PAYLOAD_IN);

    // return &pHal->bno08x_i2c_hal;
}