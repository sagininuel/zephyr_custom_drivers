# BNO08x I2C Driver for Zephyr RTOS: Just Messing Around!

## Getting a Fancy 9-Axis Sensor Working on Zephyr

### What is this?

Hey there! This is a simple, **out-of-tree (OOT)** driver I whipped up for the **CEVA BNO08x** family of sensors. Specifically, I've gotten it running great with the **BNO085** and the slightly newer **BNO086** chips!

Turns out, there wasn't a ready-to-go I2C driver for **Zephyr RTOS**, so I spent some time cracking open the datasheet and implementing the proprietary **SH-2 Host Communication Protocol** myself. This is the code you need to talk to the secret, smart firmware running inside the sensor! Now we can finally grab that sweet, sweet **Quaternion** (orientation) data in our Zephyr projects.

---

## The Techy Bits (Proof It Actually Works!)

I focused on making this driver super clean and easy to use, showing off some cool Zephyr features:

* **I2C & SH-2 Protocol:** The low-level code to handle the I2C chatting and the specific **SH-2 messaging rules**.
* **Proper Zephyr Integration:** It hooks right into Zephyr using the custom compatible string `custom,bno08x` in the **Device Tree (DTS)** and uses standard **Kconfig**.
* **Real-Time Data:** It's set up to grab that precise **Rotation Vector** data for things like robots, drones, or maybe just making an LED blink when you shake the board!

---

## Compatibility: It Runs on Nordic Stuff!

I've tested this driver on some popular boards, so it should be solid for your next project:

| Sensor Chip | Status | My Two Cents |
| :--- | :--- | :--- |
| **BNO085 & BNO086** | Works Great! | Confirmed running perfectly on the I2C bus. |
| **Nordic nRF52840 DK** | Verified! | Awesome for low-power Bluetooth LE projects. |
| **Nordic nRF7002 DK** | Verified! | Proves it's stable even when sharing resources with Wi-Fi 6! |

---

## How to Get It Running (The Setup)

This is an **Out-of-Tree** module, so follow these simple steps to pull it into your Zephyr workspace:

1.  **Clone it:** Put this folder inside your Zephyr application's `modules` folder (or wherever you keep your custom drivers).
    ```bash
    https://github.com/sagininuel/zephyr_custom_drivers.git
    ```
2.  **Edit `prj.conf`:** You need to tell Zephyr what features to turn on:
    ```kconfig
    CONFIG_BNO08X_DRIVER=y 
    CONFIG_SENSOR=y
    CONFIG_I2C=y
    CONFIG_GPIO=y 
    ```
3.  **Tweak the DTS (The Critical Part!)**

    Use an **overlay file** to tell Zephyr exactly which I2C bus and GPIO pin you've wired the sensor's interrupt pin to.

    > **⚠️ BNO086 Users Warning:**
    > The default I2C address for this driver is `0x4a`. If you are using a BNO086 breakout board, you often need to **solder the address select pins** (usually labeled AD0/AD1 or similar) to allow the sensor to use this address! Check your specific breakout board documentation!

    ```dts
    // Example for nRF52840dk
    &i2c0 {
        status = "okay";
        bno08x_sensor: bno08x@4a {
            // NOTE: Using the custom compatible name here
            compatible = "custom,bno08x"; 
            reg = <0x4a>; 
            status = "okay";
            // IMPORTANT: Change 28 to your actual Host Interrupt pin!
            int-gpios = <&gpio0 28 GPIO_PULL_UP>; 
        };
    };
    ```

---


Get it done~!


