/**
 * Copyright (c) 2025 Remantek Inc.
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#include <bno08x_platform.h>


LOG_MODULE_REGISTER(bno08x_i2c_hal, LOG_LEVEL_INF);

static const struct i2c_dt_spec * pHal_i2c;
static bool _reset_occurred = false;

typedef enum {
    TIME_MS,        // Milliseconds since boot
    TIME_US,        // Microseconds
    TIME_TICKS,     // System ticks
    TIME_CYCLES     // CPU cycles
} time_unit_t;

uint32_t getTimeNow(time_unit_t unit) {
    switch (unit) {
        case TIME_MS:
            return (uint32_t)k_uptime_get();
            
        case TIME_US:
            return k_cyc_to_us_floor32(k_cycle_get_32());
            
        case TIME_TICKS:
            return (uint32_t)k_uptime_ticks();
            
        case TIME_CYCLES:
            return k_cycle_get_32();
            
        default:
            LOG_ERR("Invalid time unit");
            return 0;
    }
}

static uint32_t getTimeUs(sh2_Hal_t * self)
{
    return getTimeNow(TIME_US);
}

static bool i2c_read_bytes(uint8_t *buffer, size_t len) {
    int ret = i2c_read_dt(pHal_i2c, buffer, len);
    if (ret != 0) {
        LOG_ERR("I2C read failed: %d", ret);
        return false;
    }
    return true;
}

static size_t i2c_get_max_buffer_size(void) {
    return SH2_HAL_MAX_PAYLOAD_IN;
}

bool wasReset(void)
{
    bool x = _reset_occurred;
    _reset_occurred = false;

    return x;
}

static int i2chal_open(sh2_Hal_t *self) {
    bool success = false;
    int ret;

    LOG_DBG("I2C HAL open");
      
    // Check if I2C device is ready
    if (pHal_i2c == NULL || !i2c_is_ready_dt(pHal_i2c)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
    
    for (uint8_t attempts = 0; attempts < 5; attempts++) {
        ret = i2c_write_dt(pHal_i2c, softreset_pkt, sizeof(softreset_pkt));
        if (ret == 0) {
            success = true;
            break;
        }
        LOG_WRN("I2C write attempt %d failed: %d", attempts + 1, ret);
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
    
    uint8_t header[4];
    if (!i2c_read_bytes(header, 4)) {
        LOG_ERR("Failed to read I2C header");
        return 0;
    }
    
    // Determine amount to read
    uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
    // Unset the "continue" bit
    packet_size &= ~0x8000;
    
    LOG_DBG("Read SHTP header. Packet size: %u & buffer size: %u", 
            packet_size, len);
    
    size_t i2c_buffer_max = i2c_get_max_buffer_size();
    if (packet_size > len) {
        // packet wouldn't fit in our buffer
        LOG_ERR("Packet size %u exceeds buffer size %u", packet_size, len);
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
        
        LOG_DBG("Reading from I2C: %u", read_size);
        LOG_DBG("Remaining to read: %u", cargo_remaining);
        
        if (!i2c_read_bytes(i2c_buffer, read_size)) {
            LOG_ERR("Failed to read I2C data");
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
    
    // Optional: Log the received data for debugging
    // #if LOG_LEVEL >= LOG_LEVEL_DBG
    if (packet_size > 0){
        LOG_DBG("Received packet data:");
        for (int i = 0; i < packet_size; i++) {
            printk("%02X ", (pBuffer - packet_size)[i]);
            if (i % 16 == 15) printk("\n");
        }
        printk("\n");
    }
    // #endif
    
    // Set timestamp if requested
    if (t_us != NULL) {
        *t_us = k_uptime_get_32() * 1000; // Convert ms to us
    }
    
    return packet_size;
}

static int i2chal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) 
{
    LOG_DBG("I2C HAL write, length: %u", len);
    
    if (!pBuffer || len == 0) {
        LOG_ERR("Invalid write parameters");
        return 0;
    }
    
    if (!i2c_is_ready_dt(pHal_i2c)) {
        LOG_ERR("I2C device not ready");
        return 0;
    }

    // Log packet for debugging
    LOG_HEXDUMP_DBG(pBuffer, MIN(len, 16), "TX Packet:");

    // Use standard I2C write (no register address for BNO085)
    int ret = i2c_write_dt(pHal_i2c, pBuffer, len);
    if (ret != 0) {
        LOG_ERR("I2C write failed: %d", ret);
        return 0;
    }

    LOG_DBG("Successfully wrote %u bytes", len);
    return len;
}

int i2c_hal_init(size_t max_buffer_size_in) {  
    if (!i2c_is_ready_dt(pHal_i2c)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }
    
    LOG_INF("I2C HAL initialized successfully");
    LOG_INF("I2C device: %s, address: 0x%02X", 
            pHal_i2c->bus->name, pHal_i2c->addr);
    return 0;
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
    pHal->sh2_Hal.getTimeUs = getTimeUs;

    i2c_hal_init(SH2_HAL_MAX_PAYLOAD_IN);
}