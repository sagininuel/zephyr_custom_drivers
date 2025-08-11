# SPDX-License-Identifier: Apache-2.0

message("CUSTOM_VL53L0X Sensor ...")
zephyr_library()
zephyr_library_sources(src/custom_vl53l0x.c)

#zephyr_library_sources_ifdef(CONFIG_VL53 src/main.c)

