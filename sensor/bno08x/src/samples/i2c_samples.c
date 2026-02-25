/*
 * Author: Gini
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/byteorder.h>

// Define the maximum expected transfer size.
// This should be set based on the specific CEVA sensor's documentation
// for its maximum I2C transfer size (e.g., 32, 128, or 256 bytes).
#define MAX_SHTP_TRANSFER_SIZE 256
#define SHTP_HEADER_SIZE 4

/**
 * @brief Reads the maximum possible data from an I2C slave in one continuous transaction.
 *
 * This function is optimized for protocols like SHTP where the slave immediately
 * transmits data (header first) upon receiving its address for a read.
 *
 * @param spec Pointer to the I2C device tree specification.
 * @param read_buf Buffer to store the incoming data (must be at least MAX_SHTP_TRANSFER_SIZE).
 * @return The number of actual payload bytes read (excluding header), or a negative error code.
 */
int i2c_read_max_transfer(const struct i2c_dt_spec *spec, uint8_t *read_buf)
{
    // Ensure the bus device is ready
    if (!device_is_ready(spec->bus)) {
        printk("I2C bus device not ready: %s\n", spec->bus->name);
        return -ENODEV;
    }

    // 1. Define the I2C Message Structure
    // This message requests the maximum possible data in one go.
    struct i2c_msg msg = {
        .buf = read_buf,
        .len = MAX_SHTP_TRANSFER_SIZE,
        // Flags: I2C_MSG_READ (for reading) | I2C_MSG_STOP (ends the transaction cleanly)
        .flags = I2C_MSG_READ | I2C_MSG_STOP
    };

    // 2. Define the message array (only one message in this case)
    struct i2c_msg msgs[] = { msg };

    // 3. Execute the single, continuous transfer
    // Sequence: START, ADDR+R, [MAX_SHTP_TRANSFER_SIZE data bytes], STOP
    int ret = i2c_transfer_dt(spec, msgs, ARRAY_SIZE(msgs));

    if (ret < 0) {
        printk("I2C read transfer failed (Error: %d)\n", ret);
        return ret;
    }

    // --- SHTP Protocol Parsing ---
    // The first 4 bytes contain the SHTP header:
    // Byte 0: Length LSB
    // Byte 1: Length MSB (MSB is continuation bit)
    // Byte 2: Channel
    // Byte 3: Sequence Number

    if (MAX_SHTP_TRANSFER_SIZE < SHTP_HEADER_SIZE) {
        // Should not happen, but for safety
        return -EINVAL;
    }

    // Extract the length field (Bytes 0 and 1)
    uint16_t raw_length = sys_get_le16(&read_buf[0]);

    // Mask out the Continuation Bit (MSB of the length field is usually a continuation flag)
    // The actual payload length is usually calculated from the first 15 bits.
    // Length is the total size (header + cargo)
    uint16_t total_shtp_length = raw_length & 0x7FFF;

    // Check for a NULL header (length 0 indicates no data available)
    if (total_shtp_length == 0) {
        // This is a common way the SHTP device signals IDLE or NO DATA.
        printk("SHTP: Received null header (no cargo).\n");
        return 0;
    }

    // Validate the length against the buffer size
    if (total_shtp_length > MAX_SHTP_TRANSFER_SIZE) {
        printk("SHTP Error: Reported length (%u) exceeds max transfer size (%u).\n",
               total_shtp_length, MAX_SHTP_TRANSFER_SIZE);
        // This might indicate an error or requires a segmented read (more complex)
        return -EIO;
    }

    // The actual payload size is the total length minus the 4-byte header
    size_t payload_size = total_shtp_length - SHTP_HEADER_SIZE;

    printk("SHTP Packet received: Total Length=%u, Payload Size=%u, Channel=0x%02x\n",
           total_shtp_length, payload_size, read_buf[2]);

    // The read_buf now contains the full packet. The caller can access
    // the payload starting at read_buf[SHTP_HEADER_SIZE].

    return (int)payload_size;
}

// --- Example Usage Context (Simulated) ---

// Assuming your I2C device is defined in the devicetree under an alias:
// #define SENSOR_NODE DT_ALIAS(my_shtp_sensor)
// const struct i2c_dt_spec shtp_device = I2C_DT_SPEC_GET(SENSOR_NODE);

/*
void main(void)
{
    uint8_t rx_buffer[MAX_SHTP_TRANSFER_SIZE];

    if (!device_is_ready(shtp_device.bus)) {
        printk("I2C bus not ready.\n");
        return;
    }

    // In a real application, you would assert the WAKE pin or poll the HINT pin
    // before calling the read function.

    int payload_len = i2c_read_max_transfer(&shtp_device, rx_buffer);

    if (payload_len > 0) {
        printk("Successfully processed SHTP data. First byte of payload: 0x%02x\n",
               rx_buffer[SHTP_HEADER_SIZE]);
    } else if (payload_len == 0) {
        // No data, successful read.
    } else {
        printk("Failed to read SHTP data.\n");
    }
}
*/
