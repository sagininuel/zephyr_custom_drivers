/*
 * Author: Gini
 * SPDX-License-Identifier: Apache-2.0
 */

//Implementation of the BNO08X i2c Hardware Abstraction Layer
#include <bno08x_platform.h>

// static const struct device *i2c_device;
static bool _reset_occurred = false;

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

// #include <zephyr/sys/time_units.h>

LOG_MODULE_REGISTER(bno08x_i2c_hal_init, LOG_LEVEL_DBG);

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

// #define SHTP_CHANNEL_EXECUTABLE 1

// int bno08x_read_packet(const struct i2c_dt_spec *pHal_i2c) {
//     uint8_t rx_buf[16];
//     int ret;

//     LOG_DBG("Probing I2C bus for device...");
//     ret = i2c_probe_dt(pHal_i2c);
//     if (ret != 0) {
//         LOG_ERR("I2C probe failed (%d). Please check:", ret);
//         LOG_ERR("1. The I2C address in your device tree is correct (default is 0x4B).");
//         LOG_ERR("2. The sensor is powered and properly connected (VCC, GND).");
//         LOG_ERR("3. You have pull-up resistors (e.g., 4.7kOhm) on the SDA and SCL lines.");
//         LOG_ERR("4. The SDA and SCL wires are not swapped.");
//         return ret;
//     }
//     // return 0;

//     ret = i2c_read_dt(pHal_i2c, rx_buf, sizeof(rx_buf));
//     if (ret != 0) {
//         LOG_WRN("Initial I2C read failed: %d", ret);
//         return ret;
//     }
//     return 0;
// }

#define BNO08X_HEADER_SIZE          4
#define CHANNEL_EXECUTABLE          1  /* Channel for executable commands */
#define SOFT_RESET_COMMAND          1  /* Command ID for soft reset */

/**
 * @brief Perform a soft reset on BNO08x sensor
 * 
 * This function sends a soft reset command to the BNO08x sensor using the
 * SH-2 protocol over I2C. The soft reset reinitializes the sensor without
 * requiring a hardware reset.
 * 
 * @param i2c_spec Pointer to I2C device tree specification
 * @return 0 on success, negative error code on failure
 */
int bno08x_soft_reset(const struct i2c_dt_spec *i2c_spec)
{
    uint8_t packet[BNO08X_HEADER_SIZE + 1]; /* Header + 1 byte command */
    uint16_t packet_length = sizeof(packet);
    int ret;

    if (!i2c_spec) {
        LOG_ERR("Invalid I2C spec pointer");
        return -EINVAL;
    }

    if (!device_is_ready(i2c_spec->bus)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    LOG_INF("Performing BNO08x soft reset");

    /* Build SH-2 packet header */
    sys_put_le16(packet_length, &packet[0]);    /* Packet length (little endian) */
    packet[2] = CHANNEL_EXECUTABLE;             /* Channel number */
    packet[3] = 0;                             /* Sequence number (can start at 0) */

    /* Add soft reset command */
    packet[4] = SOFT_RESET_COMMAND;            /* Soft reset command ID */

    /* Log packet contents for debugging */
    LOG_DBG("Sending soft reset packet:");
    LOG_HEXDUMP_DBG(packet, packet_length, "TX:");

    /* Send the packet over I2C */
    ret = i2c_write_dt(i2c_spec, packet, packet_length);
    if (ret != 0) {
        LOG_ERR("Failed to send soft reset command: %d", ret);
        return ret;
    }

    LOG_INF("Soft reset command sent successfully");
    
    /* Wait for sensor to complete reset (typical reset time is ~300ms) */
    k_sleep(K_MSEC(500));
    
    LOG_INF("Soft reset completed");
    return 0;
}


int bno08x_soft_reset_with_verification(const struct i2c_dt_spec *i2c_spec)
{
    uint8_t test_read[4];
    int ret;

    if (!i2c_spec) {
        LOG_ERR("Invalid I2C spec pointer");
        return -EINVAL;
    }

    if (!device_is_ready(i2c_spec->bus)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    /* Try to read a few bytes to verify I2C communication */
    LOG_DBG("Verifying I2C communication...");
    ret = i2c_read_dt(i2c_spec, test_read, sizeof(test_read));
    if (ret != 0) {
        LOG_WRN("I2C verification failed: %d (this may be normal if sensor is not initialized)", ret);
        /* Continue anyway - sensor might not respond until after reset */
    } else {
        LOG_DBG("I2C communication verified");
        LOG_HEXDUMP_DBG(test_read, sizeof(test_read), "Initial read:");
    }

    /* Perform the soft reset */
    return bno08x_soft_reset(i2c_spec);
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
    
    LOG_DBG("I2C device ready!");

    // ret = bno08x_read_packet(pHal_i2c);
    // ret = bno08x_soft_reset_with_verification(pHal_i2c);


    // LOG_DBG("Reset complete, going back...");

    // return ret;


    // Future Debug
    // if (ret != 0) {
    //     LOG_WRN("Initial I2C read failed. Attempting write anyway...");
    // }

    // Data Memory Barrier
    // __DMB();

    uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
    // uint8_t softreset_pkt[] = {
    //     0x01, // LSB of payload size
    //     0x00, // MSB of payload size
    //     SHTP_CHANNEL_EXECUTABLE, // Channel number
    //     0x00, // Reserved
    //     0x01  // Soft reset command (0x01)
    // };
    
    for (uint8_t attempts = 0; attempts < 5; attempts++) {
        LOG_DBG("Loop cycle: %d", attempts);
        // ret = i2c_burst_write_dt(pHal_i2c, 1, softreset_pkt, sizeof(softreset_pkt));
        ret = i2c_write_dt(pHal_i2c, softreset_pkt, sizeof(softreset_pkt));
        // ret = i2c_write_dt(pHal_i2c, softreset_pkt, sizeof(softreset_pkt));
        // ret = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_SYS_TRIGGER, BNO055_COMMAND_RESET);
        if (ret == 0) {
            success = true;
            break;
        }
        LOG_WRN("I2C write attempt %d failed: %d", attempts + 1, ret);
        // k_msleep(30);
    }

    // __DMB();
    
    if (!success) {
        LOG_ERR("Failed to send soft reset after 5 attempts");
        return -EIO;
    }
    
    k_msleep(300);
    return 0;
}


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

static void i2chal_close(sh2_Hal_t *self) {
    LOG_DBG("I2C HAL close");
    // In Zephyr, typically no explicit close needed for I2C
}

// Helper function to read from I2C device
static bool i2c_read_bytes(uint8_t *buffer, size_t len) {
    // int ret = i2c_read(i2c_device.i2c_dev, buffer, len, i2c_device.device_addr);
    int ret = i2c_read_dt(pHal_i2c, buffer, len);
    if (ret != 0) {
        LOG_ERR("I2C read failed: %d", ret);
        return false;
    }
    return true;
}

static size_t i2c_get_max_buffer_size(void) {
    // return i2c_device.max_buffer_size;
    return SH2_HAL_MAX_PAYLOAD_IN;
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
    #if LOG_LEVEL >= LOG_LEVEL_DBG
    LOG_DBG("Received packet data:");
    for (int i = 0; i < packet_size; i++) {
        printk("%02X ", (pBuffer - packet_size)[i]);
        if (i % 16 == 15) printk("\n");
    }
    printk("\n");
    #endif
    
    // Set timestamp if requested
    if (t_us != NULL) {
        *t_us = k_uptime_get_32() * 1000; // Convert ms to us
    }
    
    return packet_size;
}

static int i2chal_read_good(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) 
{
    LOG_DBG("I2C HAL read");
    
    if (!i2c_is_ready_dt(pHal_i2c)) {
        LOG_ERR("I2C device not ready");
        return 0;
    }

    uint8_t header[4];
    
    // Read the 4-byte header first using standard I2C read (no register address)
    int ret = i2c_read_dt(pHal_i2c, header, sizeof(header));
    if (ret != 0) {
        LOG_ERR("Failed to read I2C header: %d", ret);
        return 0;
    }

    // Log header for debugging
    LOG_HEXDUMP_DBG(header, sizeof(header), "Header:");

    // Determine packet size from header
    uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
    
    // Check for continuation bit (bit 15)
    bool continuation = (packet_size & 0x8000) != 0;
    
    // Unset the "continue" bit to get actual packet size
    packet_size &= ~0x8000;
    
    LOG_DBG("Packet size: %u, buffer size: %u, continuation: %s", 
            packet_size, len, continuation ? "true" : "false");

    // Validate packet size
    if (packet_size == 0) {
        LOG_DBG("Empty packet received");
        return 0;
    }
    
    if (packet_size > len) {
        LOG_ERR("Packet size %u exceeds buffer size %u", packet_size, len);
        return 0;
    }

    // Check if this is just a header-only packet
    if (packet_size <= 4) {
        memcpy(pBuffer, header, packet_size);
        LOG_DBG("Header-only packet, size: %u", packet_size);
        
        // Set timestamp if requested
        if (t_us) {
            *t_us = k_cyc_to_us_ceil32(k_cycle_get_32());
        }
        return packet_size;
    }

    // Copy header to output buffer
    memcpy(pBuffer, header, sizeof(header));
    
    // Calculate remaining bytes to read (excluding the header we already read)
    uint16_t remaining_bytes = packet_size - sizeof(header);
    size_t i2c_buffer_max = i2c_hal_cfg.max_buffer_size_in;
    uint8_t i2c_buffer[i2c_buffer_max];
    uint8_t *pBuffer_current = pBuffer + sizeof(header);
    
    LOG_DBG("Reading remaining %u bytes in chunks", remaining_bytes);

    // Read remaining data in chunks
    while (remaining_bytes > 0) {
        uint16_t chunk_size = MIN(i2c_buffer_max, remaining_bytes);
        
        LOG_DBG("Reading chunk: %u bytes, remaining: %u", chunk_size, remaining_bytes);
        
        ret = i2c_read_dt(pHal_i2c, i2c_buffer, chunk_size);
        if (ret != 0) {
            LOG_ERR("I2C chunk read failed: %d", ret);
            return 0;
        }
        
        // Copy chunk to output buffer
        memcpy(pBuffer_current, i2c_buffer, chunk_size);
        
        // Update pointers and counters
        pBuffer_current += chunk_size;
        remaining_bytes -= chunk_size;
    }

    LOG_DBG("Successfully read complete packet of %u bytes", packet_size);
    LOG_HEXDUMP_DBG(pBuffer, MIN(packet_size, 16), "Packet data:");

    // Set timestamp if requested
    if (t_us) {
        *t_us = k_cyc_to_us_ceil32(k_cycle_get_32());
    }

    return packet_size;
}

static int i2chal_read_2(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) 
{
    LOG_DBG("I2C HAL read");
    
    if (!i2c_is_ready_dt(pHal_i2c)) {
        LOG_ERR("I2C device not ready");
        return 0;
    }

    if (!pBuffer || len == 0) {
        LOG_ERR("Invalid buffer parameters");
        return 0;
    }

    uint8_t header[4];
    int ret;
    
    // Clear header buffer first
    memset(header, 0, sizeof(header));
    
    // Read the 4-byte header
    ret = i2c_read_dt(pHal_i2c, header, sizeof(header));
    if (ret != 0) {
        LOG_ERR("Failed to read I2C header: %d", ret);
        return 0;
    }

    // Always log the raw header bytes for debugging
    LOG_HEXDUMP_DBG(header, sizeof(header), "Raw header:");

    // Parse packet length (little endian)
    uint16_t raw_length = sys_get_le16(header);
    uint16_t packet_size = raw_length & 0x7FFF;  // Clear continuation bit
    bool continuation = (raw_length & 0x8000) != 0;
    
    uint8_t channel = header[2];
    uint8_t sequence = header[3];
    
    LOG_DBG("Parsed header - Raw: 0x%04X, Size: %u, Channel: %u, Seq: %u, Cont: %s",
            raw_length, packet_size, channel, sequence, continuation ? "Y" : "N");

    // Validate packet size - BNO085 packets should be reasonable size
    if (packet_size == 0) {
        LOG_DBG("Zero-length packet received");
        return 0;
    }
    
    if (packet_size > 255) {  // BNO085 max packet size is typically ~255 bytes
        LOG_ERR("Invalid packet size: %u (raw header: 0x%02X 0x%02X 0x%02X 0x%02X)",
                packet_size, header[0], header[1], header[2], header[3]);
        LOG_ERR("This suggests I2C communication problems:");
        LOG_ERR("- Check pull-up resistors (4.7kΩ on SDA/SCL)");
        LOG_ERR("- Verify power supply stability (3.3V)"); 
        LOG_ERR("- Check I2C clock speed (try 100kHz instead of 400kHz)");
        LOG_ERR("- Verify device address (should be 0x4A or 0x4B)");
        return 0;
    }
    
    if (packet_size > len) {
        LOG_ERR("Packet size %u exceeds buffer size %u", packet_size, len);
        return 0;
    }

    // Validate channel number (BNO085 uses channels 0-5)
    if (channel > 5) {
        LOG_WRN("Unexpected channel number: %u", channel);
    }

    // Copy header to output buffer
    memcpy(pBuffer, header, sizeof(header));
    
    // If packet is header-only, we're done
    if (packet_size <= sizeof(header)) {
        LOG_DBG("Header-only packet, size: %u", packet_size);
        if (t_us) {
            *t_us = k_cyc_to_us_ceil32(k_cycle_get_32());
        }
        return packet_size;
    }

    // Read the remaining payload
    uint16_t payload_size = packet_size - sizeof(header);
    LOG_DBG("Reading payload: %u bytes", payload_size);
    
    ret = i2c_read_dt(pHal_i2c, pBuffer + sizeof(header), payload_size);
    if (ret != 0) {
        LOG_ERR("Failed to read payload: %d", ret);
        return 0;
    }

    // Log some payload data for debugging
    if (payload_size > 0) {
        LOG_HEXDUMP_DBG(pBuffer + sizeof(header), MIN(payload_size, 8), "Payload start:");
    }

    LOG_DBG("Successfully read packet: %u bytes total", packet_size);

    if (t_us) {
        *t_us = k_cyc_to_us_ceil32(k_cycle_get_32());
    }

    return packet_size;
}

/* Enhanced version with I2C bus recovery */
static int i2chal_read_with_recovery(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) 
{
    LOG_DBG("I2C HAL read with recovery");
    
    if (!i2c_is_ready_dt(pHal_i2c)) {
        LOG_ERR("I2C device not ready");
        return 0;
    }

    uint8_t header[4];
    int ret;
    int recovery_attempts = 0;
    const int max_recovery_attempts = 3;

retry_read:
    // Clear header buffer
    memset(header, 0, sizeof(header));
    
    // Read header
    ret = i2c_read_dt(pHal_i2c, header, sizeof(header));
    if (ret != 0) {
        LOG_ERR("Header read failed: %d", ret);
        
        // Attempt I2C bus recovery
        if (recovery_attempts < max_recovery_attempts) {
            recovery_attempts++;
            LOG_WRN("Attempting I2C recovery %d/%d", recovery_attempts, max_recovery_attempts);
            
            // Simple recovery: small delay and retry
            k_sleep(K_MSEC(10));
            goto retry_read;
        }
        
        return 0;
    }

    // Log raw header
    LOG_HEXDUMP_DBG(header, sizeof(header), "Header bytes:");

    // Check for all-zero or all-0xFF header (indicates communication issues)
    bool all_zero = true;
    bool all_ff = true;
    for (int i = 0; i < 4; i++) {
        if (header[i] != 0x00) all_zero = false;
        if (header[i] != 0xFF) all_ff = false;
    }
    
    if (all_zero) {
        LOG_WRN("Received all-zero header, sensor may not be ready");
        return 0;
    }
    
    if (all_ff) {
        LOG_WRN("Received all-0xFF header, possible I2C bus issue");
        return 0;
    }

    // Parse length with additional validation
    uint16_t raw_length = (uint16_t)header[0] | ((uint16_t)header[1] << 8);
    uint16_t packet_size = raw_length & 0x7FFF;
    
    LOG_DBG("Raw length bytes: 0x%02X 0x%02X = 0x%04X = %u", 
            header[0], header[1], raw_length, packet_size);

    // Strict validation for BNO085
    if (packet_size < 4 || packet_size > 255) {
        LOG_ERR("Invalid packet size: %u", packet_size);
        LOG_ERR("Header dump: %02X %02X %02X %02X", 
                header[0], header[1], header[2], header[3]);
        
        // If this happens repeatedly, try recovery
        if (recovery_attempts < max_recovery_attempts) {
            recovery_attempts++;
            LOG_WRN("Invalid packet, attempting recovery %d/%d", 
                    recovery_attempts, max_recovery_attempts);
            k_sleep(K_MSEC(5));
            goto retry_read;
        }
        
        return 0;
    }
    
    if (packet_size > len) {
        LOG_ERR("Packet size %u exceeds buffer %u", packet_size, len);
        return 0;
    }

    // Continue with normal packet processing
    memcpy(pBuffer, header, sizeof(header));
    
    if (packet_size > sizeof(header)) {
        uint16_t payload_size = packet_size - sizeof(header);
        ret = i2c_read_dt(pHal_i2c, pBuffer + sizeof(header), payload_size);
        if (ret != 0) {
            LOG_ERR("Payload read failed: %d", ret);
            return 0;
        }
    }

    if (t_us) {
        *t_us = k_cyc_to_us_ceil32(k_cycle_get_32());
    }

    return packet_size;
}

/* Version that checks for sensor readiness first */
static int i2chal_read_with_readiness_check(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) 
{
    LOG_DBG("I2C HAL read with readiness check");
    
    if (!i2c_is_ready_dt(pHal_i2c)) {
        LOG_ERR("I2C device not ready");
        return 0;
    }

    // First, check if sensor has data ready by reading just one byte
    uint8_t status_byte;
    int ret = i2c_read_dt(pHal_i2c, &status_byte, 1);
    if (ret != 0) {
        LOG_DBG("No data available from sensor: %d", ret);
        return 0;  // No data available, not an error
    }

    // If we get here, sensor responded, so read the full header
    uint8_t header[4];
    header[0] = status_byte;  // We already have the first byte
    
    // Read the remaining 3 header bytes
    ret = i2c_read_dt(pHal_i2c, &header[1], 3);
    if (ret != 0) {
        LOG_ERR("Failed to read remaining header: %d", ret);
        return 0;
    }

    LOG_HEXDUMP_DBG(header, sizeof(header), "Complete header:");

    // Parse and validate
    uint16_t packet_size = sys_get_le16(header) & 0x7FFF;
    
    if (packet_size == 0 || packet_size > 255 || packet_size > len) {
        LOG_ERR("Invalid packet size: %u", packet_size);
        return 0;
    }

    // Copy header and read payload if needed
    memcpy(pBuffer, header, sizeof(header));
    
    if (packet_size > sizeof(header)) {
        ret = i2c_read_dt(pHal_i2c, pBuffer + sizeof(header), packet_size - sizeof(header));
        if (ret != 0) {
            LOG_ERR("Payload read failed: %d", ret);
            return 0;
        }
    }

    if (t_us) {
        *t_us = k_cyc_to_us_ceil32(k_cycle_get_32());
    }

    return packet_size;
}

/* Add this function to help debug I2C issues */
int bno085_i2c_debug_scan(void)
{
    LOG_INF("BNO085 I2C Debug Scan");
    
    if (!i2c_is_ready_dt(pHal_i2c)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    LOG_INF("I2C Bus: %s, Address: 0x%02X", pHal_i2c->bus->name, pHal_i2c->addr);

    // Try to detect device at current address
    uint8_t dummy;
    int ret = i2c_read_dt(pHal_i2c, &dummy, 1);
    if (ret == 0) {
        LOG_INF("Device responds at address 0x%02X", pHal_i2c->addr);
    } else {
        LOG_ERR("No response at address 0x%02X: %d", pHal_i2c->addr, ret);
        
        // Try common BNO085 addresses
        uint8_t test_addresses[] = {0x4A, 0x4B};
        for (int i = 0; i < ARRAY_SIZE(test_addresses); i++) {
            struct i2c_dt_spec test_spec = *pHal_i2c;
            test_spec.addr = test_addresses[i];
            
            ret = i2c_read_dt(&test_spec, &dummy, 1);
            if (ret == 0) {
                LOG_INF("Found device at address 0x%02X!", test_addresses[i]);
            }
        }
    }

    return ret;
}

static int i2chal_read_(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
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

/* Version with chunked writing for large packets */
static int i2chal_write_chunked(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) 
{
    LOG_DBG("I2C HAL write (chunked), length: %u", len);
    
    if (!pBuffer || len == 0) {
        LOG_ERR("Invalid write parameters");
        return 0;
    }
    
    if (!i2c_is_ready_dt(pHal_i2c)) {
        LOG_ERR("I2C device not ready");
        return 0;
    }

    size_t i2c_buffer_max = i2c_hal_cfg.max_buffer_size_out;
    LOG_DBG("Max I2C buffer size: %zu", i2c_buffer_max);

    // Log the full packet header for debugging
    LOG_HEXDUMP_DBG(pBuffer, MIN(len, 16), "TX Packet:");

    // If packet fits in one transaction, send it all at once
    if (len <= i2c_buffer_max) {
        int ret = i2c_write_dt(pHal_i2c, pBuffer, len);
        if (ret != 0) {
            LOG_ERR("I2C write failed: %d", ret);
            return 0;
        }
        LOG_DBG("Single write completed: %u bytes", len);
        return len;
    }

    // For large packets, write in chunks
    uint8_t *current_ptr = pBuffer;
    unsigned remaining = len;
    unsigned total_written = 0;

    LOG_DBG("Splitting write into chunks (max chunk size: %zu)", i2c_buffer_max);

    while (remaining > 0) {
        unsigned chunk_size = MIN(remaining, i2c_buffer_max);
        
        LOG_DBG("Writing chunk: %u bytes, remaining: %u", chunk_size, remaining);
        
        int ret = i2c_write_dt(pHal_i2c, current_ptr, chunk_size);
        if (ret != 0) {
            LOG_ERR("I2C chunk write failed: %d", ret);
            return total_written; // Return bytes written so far
        }
        
        current_ptr += chunk_size;
        remaining -= chunk_size;
        total_written += chunk_size;
        
        // Small delay between chunks if needed
        if (remaining > 0) {
            k_sleep(K_USEC(100)); // 100 microsecond delay
        }
    }

    LOG_DBG("Chunked write completed: %u bytes total", total_written);
    return total_written;
}

/* Version with retry logic for improved reliability */
static int i2chal_write_with_retry(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) 
{
    LOG_DBG("I2C HAL write with retry, length: %u", len);
    
    if (!pBuffer || len == 0) {
        LOG_ERR("Invalid write parameters");
        return 0;
    }
    
    if (!i2c_is_ready_dt(pHal_i2c)) {
        LOG_ERR("I2C device not ready");
        return 0;
    }

    // Log packet details
    LOG_HEXDUMP_DBG(pBuffer, MIN(len, 16), "TX Packet:");
    
    // Parse SH-2 header for logging
    if (len >= 4) {
        uint16_t packet_len = sys_get_le16(&pBuffer[0]) & 0x7FFF;
        uint8_t channel = pBuffer[2];
        uint8_t sequence = pBuffer[3];
        LOG_DBG("SH-2 Header - Length: %u, Channel: %u, Sequence: %u", 
                packet_len, channel, sequence);
    }

    int retry_count = 3;
    int ret = -1;
    
    // Retry write operation if it fails
    while (retry_count-- > 0 && ret != 0) {
        ret = i2c_write_dt(pHal_i2c, pBuffer, len);
        if (ret != 0) {
            LOG_WRN("I2C write attempt failed: %d, retries left: %d", ret, retry_count);
            if (retry_count > 0) {
                k_sleep(K_MSEC(1)); // Short delay before retry
            }
        }
    }
    
    if (ret != 0) {
        LOG_ERR("I2C write failed after retries: %d", ret);
        return 0;
    }

    LOG_DBG("Write successful: %u bytes", len);
    return len;
}

/* Debug version with comprehensive error reporting */
static int i2chal_write_debug(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) 
{
    LOG_DBG("I2C HAL write (debug), length: %u", len);
    
    if (!pHal_i2c) {
        LOG_ERR("I2C device tree spec is NULL");
        return 0;
    }
    
    if (!pBuffer) {
        LOG_ERR("Buffer pointer is NULL");
        return 0;
    }
    
    if (len == 0) {
        LOG_ERR("Write length is zero");
        return 0;
    }
    
    if (!i2c_is_ready_dt(pHal_i2c)) {
        LOG_ERR("I2C device not ready");
        LOG_ERR("Check that I2C device is properly initialized");
        return 0;
    }

    // Log I2C configuration
    LOG_DBG("I2C Config - Bus: %s, Address: 0x%02X", 
            pHal_i2c->bus->name, pHal_i2c->addr);

    // Validate packet structure
    if (len < 4) {
        LOG_WRN("Packet too short for SH-2 protocol: %u bytes", len);
    } else {
        uint16_t declared_len = sys_get_le16(&pBuffer[0]) & 0x7FFF;
        if (declared_len != len) {
            LOG_WRN("Length mismatch - Header: %u, Actual: %u", declared_len, len);
        }
    }

    // Log full packet if small, or header + footer if large
    if (len <= 32) {
        LOG_HEXDUMP_DBG(pBuffer, len, "TX Full Packet:");
    } else {
        LOG_HEXDUMP_DBG(pBuffer, 16, "TX Packet Header:");
        LOG_HEXDUMP_DBG(&pBuffer[len-8], 8, "TX Packet Footer:");
    }

    // Perform the write
    int ret = i2c_write_dt(pHal_i2c, pBuffer, len);
    if (ret != 0) {
        LOG_ERR("I2C write failed with error: %d", ret);
        LOG_ERR("Possible causes:");
        LOG_ERR("- Device not responding (check address: 0x%02X)", pHal_i2c->addr);
        LOG_ERR("- Bus busy or locked");
        LOG_ERR("- Hardware connection issues");
        LOG_ERR("- Insufficient power supply");
        LOG_ERR("- Missing pull-up resistors");
        return 0;
    }

    LOG_DBG("I2C write successful: %u bytes", len);
    return len;
}

/* Optimized version for performance-critical applications */
static int i2chal_write_fast(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) 
{
    // Minimal error checking for speed
    if (!i2c_is_ready_dt(pHal_i2c) || !pBuffer || len == 0) {
        return 0;
    }

    // Direct I2C write without extensive logging
    int ret = i2c_write_dt(pHal_i2c, pBuffer, len);
    if (ret != 0) {
        LOG_ERR("Fast write failed: %d", ret);
        return 0;
    }

    return len;
}

/* Example of how to choose the right write function based on your needs */
static int i2chal_write_adaptive(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) 
{
    // Choose write method based on packet size and system configuration
    size_t max_buffer = i2c_hal_cfg.max_buffer_size_out;
    
    if (len > max_buffer) {
        // Use chunked write for large packets
        return i2chal_write_chunked(self, pBuffer, len);
    } else if (IS_ENABLED(CONFIG_LOG_DEFAULT_LEVEL) && 
               CONFIG_LOG_DEFAULT_LEVEL >= LOG_LEVEL_DBG) {
        // Use debug version if debug logging is enabled
        return i2chal_write_debug(self, pBuffer, len);
    } else {
        // Use standard version for normal operation
        return i2chal_write(self, pBuffer, len);
    }
}

static int i2chal_write_(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    LOG_DBG("I2C Hal write");

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
// static int i2chal_read_reg(sh2_Hal_t *self, uint8_t reg_addr, 
//                           uint8_t *pBuffer, unsigned int len, uint32_t *t_us) {
//     LOG_DBG("I2C HAL read from register 0x%02X", reg_addr);
    
//     if (!i2c_is_ready_dt(pHal_i2c)) {
//         LOG_ERR("I2C device not ready");
//         return 0;
//     }
    
//     int ret = i2c_burst_read_dt(pHal_i2c, reg_addr, pBuffer, len);
//     if (ret != 0) {
//         LOG_ERR("Failed to read from register 0x%02X: %d", reg_addr, ret);
//         return 0;
//     }
    
//     // Set timestamp if requested
//     // if (t_us) {
//     //     *t_us = k_cyc_to_us_ceil32(k_cycle_get_32());
//     // }
    
//     return len;
// }

// static int i2chal_write_reg(sh2_Hal_t *self, uint8_t reg_addr, 
//                            uint8_t *pBuffer, unsigned int len) {
//     LOG_DBG("I2C HAL write to register 0x%02X", reg_addr);
    
//     if (!i2c_is_ready_dt(pHal_i2c)) {
//         LOG_ERR("I2C device not ready");
//         return 0;
//     }
    
//     int ret = i2c_burst_write_dt(pHal_i2c, reg_addr, pBuffer, len);
//     if (ret != 0) {
//         LOG_ERR("Failed to write to register 0x%02X: %d", reg_addr, ret);
//         return 0;
//     }
    
//     return len;
// }


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

    // Check i2c peripheral is ready.
    i2c_hal_init(SH2_HAL_MAX_PAYLOAD_IN);

    // return &pHal->bno08x_i2c_hal;
}