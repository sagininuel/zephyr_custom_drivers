# SPDX-License-Identifier: Apache-2.0

message("CUSTOM_BNO08X ...")
zephyr_library()
zephyr_library_sources(src/custom_bno08x.c)
zephyr_library_sources(src/bno08x_i2c_hal.c)

# Explicitly add the include directory
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inc)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/api/sh2)

#zephyr_library_sources_ifdef(CONFIG_BNO086 src/main.c)

