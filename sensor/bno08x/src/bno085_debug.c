#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(bno085_debug, CONFIG_SENSOR_LOG_LEVEL);

/* BNO085 I2C Communication Protocol */
#define BNO085_I2C_ADDRESS          0x4A
#define BNO085_HEADER_SIZE          4
#define BNO085_MAX_PACKET_SIZE      255

/* SH-2 Channel IDs */
#define CHANNEL_CONTROL             0
#define CHANNEL_EXECUTABLE          1  
#define CHANNEL_COMMAND             2
#define CHANNEL_REPORTS             3

/* SH-2 Report IDs */
#define SHTP_REPORT_COMMAND_RESPONSE    0xF1
#define SHTP_REPORT_COMMAND_REQUEST     0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE   0xF3
#define SHTP_REPORT_FRS_READ_REQUEST    0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST  0xF9
#define SHTP_REPORT_BASE_TIMESTAMP      0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

/* Commands */
#define COMMAND_ERRORS              1
#define COMMAND_COUNTER             2
#define COMMAND_TARE                3
#define COMMAND_INITIALIZE          4
#define COMMAND_DCD                 6
#define COMMAND_ME_CALIBRATE        7
#define COMMAND_DCD_PERIOD_SAVE     9
#define COMMAND_OSCILLATOR          10
#define COMMAND_CLEAR_DCD           11

/* Report IDs for sensor data */
#define SENSOR_REPORTID_ACCELEROMETER           0x01
#define SENSOR_REPORTID_GYROSCOPE              0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD         0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION    0x04
#define SENSOR_REPORTID_ROTATION_VECTOR        0x05
#define SENSOR_REPORTID_GRAVITY                0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR   0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09

struct bno085_data {
    const struct device *i2c_dev;
    uint8_t i2c_addr;
    uint16_t packet_sequence_number[6]; // One for each channel
    uint8_t rx_buffer[BNO085_MAX_PACKET_SIZE];
    uint8_t tx_buffer[BNO085_MAX_PACKET_SIZE];
    struct k_mutex lock;
};

struct bno085_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec reset_gpio;
    struct gpio_dt_spec int_gpio;
};

/* Function prototypes */
static int bno085_read_packet(const struct device *dev, uint8_t *buffer, uint16_t *packet_length);
static int bno085_send_packet(const struct device *dev, uint8_t channel, uint8_t *data, uint16_t length);
static int bno085_wait_for_int(const struct device *dev);
static int bno085_soft_reset(const struct device *dev);
static int bno085_get_product_id(const struct device *dev);
static int bno085_enable_report(const struct device *dev, uint8_t report_id, uint32_t interval_us);

static int bno085_read_packet(const struct device *dev, uint8_t *buffer, uint16_t *packet_length)
{
    struct bno085_data *data = dev->data;
    const struct bno085_config *cfg = dev->config;
    uint8_t header[BNO085_HEADER_SIZE];
    int ret;

    /* Read the 4-byte header first */
    ret = i2c_read_dt(&cfg->i2c, header, BNO085_HEADER_SIZE);
    if (ret != 0) {
        LOG_ERR("Failed to read packet header: %d", ret);
        return ret;
    }

    /* Parse header */
    uint16_t length = sys_get_le16(&header[0]) & 0x7FFF;
    uint8_t channel = header[2];
    uint8_t sequence = header[3];

    LOG_DBG("Header: length=%d, channel=%d, sequence=%d", length, channel, sequence);

    if (length == 0 || length > BNO085_MAX_PACKET_SIZE) {
        LOG_ERR("Invalid packet length: %d", length);
        return -EINVAL;
    }

    /* Copy header to buffer */
    memcpy(buffer, header, BNO085_HEADER_SIZE);
    *packet_length = BNO085_HEADER_SIZE;

    /* Read remaining data if any */
    if (length > BNO085_HEADER_SIZE) {
        uint16_t remaining = length - BNO085_HEADER_SIZE;
        ret = i2c_read_dt(&cfg->i2c, &buffer[BNO085_HEADER_SIZE], remaining);
        if (ret != 0) {
            LOG_ERR("Failed to read packet data: %d", ret);
            return ret;
        }
        *packet_length = length;
    }

    return 0;
}

static int bno085_send_packet(const struct device *dev, uint8_t channel, uint8_t *data, uint16_t length)
{
    struct bno085_data *drv_data = dev->data;
    const struct bno085_config *cfg = dev->config;
    uint8_t *buffer = drv_data->tx_buffer;
    uint16_t total_length = length + BNO085_HEADER_SIZE;
    int ret;

    if (total_length > BNO085_MAX_PACKET_SIZE) {
        LOG_ERR("Packet too large: %d", total_length);
        return -EINVAL;
    }

    /* Build header */
    sys_put_le16(total_length, &buffer[0]);
    buffer[2] = channel;
    buffer[3] = drv_data->packet_sequence_number[channel]++;

    /* Copy data */
    if (length > 0) {
        memcpy(&buffer[BNO085_HEADER_SIZE], data, length);
    }

    LOG_DBG("Sending packet: channel=%d, length=%d", channel, total_length);
    LOG_HEXDUMP_DBG(buffer, total_length, "TX:");

    /* Send packet */
    ret = i2c_write_dt(&cfg->i2c, buffer, total_length);
    if (ret != 0) {
        LOG_ERR("Failed to send packet: %d", ret);
        return ret;
    }

    return 0;
}

static int bno085_wait_for_int(const struct device *dev)
{
    const struct bno085_config *cfg = dev->config;
    int timeout = 1000; // 1 second timeout
    
    if (!cfg->int_gpio.port) {
        /* If no interrupt pin configured, just wait a bit */
        k_sleep(K_MSEC(10));
        return 0;
    }

    /* Wait for interrupt pin to go low (data ready) */
    while (timeout-- > 0) {
        if (gpio_pin_get_dt(&cfg->int_gpio) == 0) {
            return 0;
        }
        k_sleep(K_MSEC(1));
    }

    LOG_WRN("Timeout waiting for interrupt");
    return -ETIMEDOUT;
}

static int bno085_soft_reset(const struct device *dev)
{
    uint8_t reset_cmd = 1; // Executable channel reset command
    int ret;

    LOG_INF("Performing soft reset");
    
    ret = bno085_send_packet(dev, CHANNEL_EXECUTABLE, &reset_cmd, 1);
    if (ret != 0) {
        LOG_ERR("Failed to send reset command: %d", ret);
        return ret;
    }

    /* Wait for reset to complete */
    k_sleep(K_MSEC(500));

    return 0;
}

static int bno085_get_product_id(const struct device *dev)
{
    struct bno085_data *data = dev->data;
    uint8_t product_id_req = 0; // No additional data needed
    uint16_t packet_length;
    int ret;

    LOG_INF("Requesting product ID");

    ret = bno085_send_packet(dev, CHANNEL_CONTROL, &product_id_req, 0);
    if (ret != 0) {
        LOG_ERR("Failed to send product ID request: %d", ret);
        return ret;
    }

    /* Wait for response */
    ret = bno085_wait_for_int(dev);
    if (ret != 0) {
        return ret;
    }

    ret = bno085_read_packet(dev, data->rx_buffer, &packet_length);
    if (ret != 0) {
        LOG_ERR("Failed to read product ID response: %d", ret);
        return ret;
    }

    if (packet_length >= 8 && data->rx_buffer[4] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
        uint32_t sw_major = sys_get_le32(&data->rx_buffer[8]);
        uint32_t sw_minor = sys_get_le32(&data->rx_buffer[12]);
        uint32_t sw_patch = sys_get_le32(&data->rx_buffer[16]);
        uint32_t sw_build = sys_get_le32(&data->rx_buffer[20]);
        
        LOG_INF("BNO085 Software Version: %d.%d.%d.%d", 
                sw_major, sw_minor, sw_patch, sw_build);
    }

    return 0;
}

static int bno085_enable_report(const struct device *dev, uint8_t report_id, uint32_t interval_us)
{
    struct bno085_data *data = dev->data;
    uint8_t feature_cmd[17];
    uint16_t packet_length;
    int ret;

    LOG_INF("Enabling report ID %d with interval %d us", report_id, interval_us);

    /* Build Set Feature Command */
    feature_cmd[0] = SHTP_REPORT_SET_FEATURE_COMMAND;
    feature_cmd[1] = report_id;
    feature_cmd[2] = 0; // Feature flags
    feature_cmd[3] = 0; // Change sensitivity (relative)
    feature_cmd[4] = 0; // Change sensitivity (relative)
    sys_put_le32(interval_us, &feature_cmd[5]); // Report interval in microseconds
    sys_put_le32(0, &feature_cmd[9]);           // Batch interval
    sys_put_le32(0, &feature_cmd[13]);          // Sensor-specific config

    ret = bno085_send_packet(dev, CHANNEL_CONTROL, feature_cmd, 17);
    if (ret != 0) {
        LOG_ERR("Failed to send enable report command: %d", ret);
        return ret;
    }

    /* Wait for command response */
    ret = bno085_wait_for_int(dev);
    if (ret != 0) {
        return ret;
    }

    ret = bno085_read_packet(dev, data->rx_buffer, &packet_length);
    if (ret != 0) {
        LOG_ERR("Failed to read command response: %d", ret);
        return ret;
    }

    if (packet_length >= 6 && data->rx_buffer[4] == SHTP_REPORT_COMMAND_RESPONSE) {
        uint8_t command = data->rx_buffer[5];
        uint8_t status = data->rx_buffer[6];
        LOG_INF("Command response: cmd=%d, status=%d", command, status);
        
        if (status != 0) {
            LOG_ERR("Command failed with status: %d", status);
            return -EIO;
        }
    }

    return 0;
}

static int bno085_init(const struct device *dev)
{
    struct bno085_data *data = dev->data;
    const struct bno085_config *cfg = dev->config;
    int ret;

    LOG_INF("Initializing BNO085");

    /* Initialize mutex */
    k_mutex_init(&data->lock);

    /* Check if I2C device is ready */
    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    data->i2c_dev = cfg->i2c.bus;
    data->i2c_addr = cfg->i2c.addr;

    /* Initialize sequence numbers */
    for (int i = 0; i < 6; i++) {
        data->packet_sequence_number[i] = 0;
    }

    /* Configure GPIO pins if available */
    if (cfg->reset_gpio.port) {
        if (!gpio_is_ready_dt(&cfg->reset_gpio)) {
            LOG_ERR("Reset GPIO not ready");
            return -ENODEV;
        }
        
        ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret != 0) {
            LOG_ERR("Failed to configure reset GPIO: %d", ret);
            return ret;
        }

        /* Perform hardware reset */
        gpio_pin_set_dt(&cfg->reset_gpio, 1);
        k_sleep(K_MSEC(10));
        gpio_pin_set_dt(&cfg->reset_gpio, 0);
        k_sleep(K_MSEC(100));
    }

    if (cfg->int_gpio.port) {
        if (!gpio_is_ready_dt(&cfg->int_gpio)) {
            LOG_ERR("Interrupt GPIO not ready");
            return -ENODEV;
        }
        
        ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
        if (ret != 0) {
            LOG_ERR("Failed to configure interrupt GPIO: %d", ret);
            return ret;
        }
    }

    /* Wait for sensor to boot */
    k_sleep(K_MSEC(500));

    /* Perform soft reset */
    ret = bno085_soft_reset(dev);
    if (ret != 0) {
        LOG_ERR("Soft reset failed: %d", ret);
        return ret;
    }

    /* Get product ID to verify communication */
    ret = bno085_get_product_id(dev);
    if (ret != 0) {
        LOG_ERR("Failed to get product ID: %d", ret);
        return ret;
    }

    /* Enable some basic reports for testing */
    ret = bno085_enable_report(dev, SENSOR_REPORTID_ROTATION_VECTOR, 100000); // 100ms interval
    if (ret != 0) {
        LOG_WRN("Failed to enable rotation vector report: %d", ret);
    }

    ret = bno085_enable_report(dev, SENSOR_REPORTID_ACCELEROMETER, 50000); // 50ms interval  
    if (ret != 0) {
        LOG_WRN("Failed to enable accelerometer report: %d", ret);
    }

    LOG_INF("BNO085 initialization complete");
    return 0;
}

/* Read sensor data - this would be called periodically */
int bno085_read_sensor_data(const struct device *dev)
{
    struct bno085_data *data = dev->data;
    uint16_t packet_length;
    int ret;

    k_mutex_lock(&data->lock, K_FOREVER);

    ret = bno085_wait_for_int(dev);
    if (ret == -ETIMEDOUT) {
        k_mutex_unlock(&data->lock);
        return 0; // No data available, not an error
    }

    ret = bno085_read_packet(dev, data->rx_buffer, &packet_length);
    if (ret != 0) {
        k_mutex_unlock(&data->lock);
        return ret;
    }

    if (packet_length >= 5) {
        uint8_t report_id = data->rx_buffer[4];
        
        switch (report_id) {
        case SENSOR_REPORTID_ROTATION_VECTOR:
            if (packet_length >= 18) {
                float i = *(float*)&data->rx_buffer[6];
                float j = *(float*)&data->rx_buffer[10];
                float k = *(float*)&data->rx_buffer[14];
                float real = *(float*)&data->rx_buffer[18];
                LOG_INF("Rotation Vector: i=%f j=%f k=%f real=%f", i, j, k, real);
            }
            break;
            
        case SENSOR_REPORTID_ACCELEROMETER:
            if (packet_length >= 16) {
                int16_t x = sys_get_le16(&data->rx_buffer[6]);
                int16_t y = sys_get_le16(&data->rx_buffer[8]);
                int16_t z = sys_get_le16(&data->rx_buffer[10]);
                LOG_INF("Accelerometer: x=%d y=%d z=%d", x, y, z);
            }
            break;
            
        default:
            LOG_DBG("Unknown report ID: 0x%02X", report_id);
            break;
        }
    }

    k_mutex_unlock(&data->lock);
    return 0;
}

/* Device tree macro helpers */
#define BNO085_CONFIG(inst)                           \
    {                                                 \
        .i2c = I2C_DT_SPEC_INST_GET(inst),           \
        .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {}), \
        .int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {}),     \
    }

#define BNO085_DEFINE(inst)                           \
    static struct bno085_data bno085_data_##inst;    \
    static const struct bno085_config bno085_config_##inst = BNO085_CONFIG(inst); \
    DEVICE_DT_INST_DEFINE(inst, bno085_init, NULL,   \
                         &bno085_data_##inst,         \
                         &bno085_config_##inst,       \
                         POST_KERNEL,                 \
                         CONFIG_SENSOR_INIT_PRIORITY, \
                         NULL);

DT_INST_FOREACH_STATUS_OKAY(BNO085_DEFINE)





//////////////////////////////////////////
// 2nd working



static int i2chal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) 
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





//////////////////////////////////////////////
//1st working

static int i2chal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) 
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

/* Alternative version with better error handling and timeout */
static int i2chal_read_with_timeout(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) 
{
    LOG_DBG("I2C HAL read with timeout");
    
    if (!pHal_i2c || !pBuffer) {
        LOG_ERR("Invalid parameters");
        return 0;
    }
    
    if (!i2c_is_ready_dt(pHal_i2c)) {
        LOG_ERR("I2C device not ready");
        return 0;
    }

    uint8_t header[4];
    int retry_count = 3;
    int ret = -1;
    
    // Retry header read if it fails
    while (retry_count-- > 0 && ret != 0) {
        ret = i2c_read_dt(pHal_i2c, header, sizeof(header));
        if (ret != 0) {
            LOG_WRN("Header read attempt failed: %d, retries left: %d", ret, retry_count);
            if (retry_count > 0) {
                k_sleep(K_MSEC(1)); // Short delay before retry
            }
        }
    }
    
    if (ret != 0) {
        LOG_ERR("Failed to read I2C header after retries: %d", ret);
        return 0;
    }

    // Parse header
    uint16_t packet_size = sys_get_le16(header); // Use Zephyr's little-endian helper
    bool continuation = (packet_size & 0x8000) != 0;
    packet_size &= ~0x8000;
    
    uint8_t channel = header[2];
    uint8_t sequence = header[3];
    
    LOG_DBG("Header - Size: %u, Channel: %u, Sequence: %u, Continuation: %s", 
            packet_size, channel, sequence, continuation ? "yes" : "no");

    // Validate packet
    if (packet_size == 0 || packet_size > len) {
        if (packet_size == 0) {
            LOG_DBG("Empty packet");
        } else {
            LOG_ERR("Packet too large: %u > %u", packet_size, len);
        }
        return 0;
    }

    // Copy header
    memcpy(pBuffer, header, sizeof(header));
    
    if (packet_size <= sizeof(header)) {
        // Header-only packet
        if (t_us) {
            *t_us = k_cyc_to_us_ceil32(k_cycle_get_32());
        }
        return packet_size;
    }

    // Read payload
    uint16_t payload_size = packet_size - sizeof(header);
    ret = i2c_read_dt(pHal_i2c, pBuffer + sizeof(header), payload_size);
    if (ret != 0) {
        LOG_ERR("Failed to read payload: %d", ret);
        return 0;
    }

    if (t_us) {
        *t_us = k_cyc_to_us_ceil32(k_cycle_get_32());
    }

    return packet_size;
}

/* Debugging version that checks I2C bus state */
static int i2chal_read_debug(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) 
{
    LOG_DBG("I2C HAL read (debug version)");
    
    if (!i2c_is_ready_dt(pHal_i2c)) {
        LOG_ERR("I2C device not ready");
        return 0;
    }

    // Check I2C bus configuration
    LOG_DBG("I2C Config - Bus: %s, Address: 0x%02X", 
            pHal_i2c->bus->name, pHal_i2c->addr);

    // Try a simple I2C detection first
    uint8_t dummy;
    int detect_ret = i2c_read_dt(pHal_i2c, &dummy, 1);
    if (detect_ret != 0) {
        LOG_ERR("I2C device not responding to read request: %d", detect_ret);
        LOG_ERR("Check connections, pull-ups, and device address");
        return 0;
    }
    
    LOG_DBG("I2C device responded, proceeding with header read");

    // Now try the actual header read
    uint8_t header[4];
    int ret = i2c_read_dt(pHal_i2c, header, sizeof(header));
    if (ret != 0) {
        LOG_ERR("Header read failed: %d", ret);
        LOG_ERR("This might indicate:");
        LOG_ERR("- Incorrect I2C address (current: 0x%02X)", pHal_i2c->addr);
        LOG_ERR("- Missing pull-up resistors");
        LOG_ERR("- Power supply issues");
        LOG_ERR("- Sensor not properly initialized");
        return 0;
    }

    // Continue with normal processing...
    uint16_t packet_size = sys_get_le16(header);
    packet_size &= ~0x8000;
    
    if (packet_size == 0 || packet_size > len) {
        return 0;
    }

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