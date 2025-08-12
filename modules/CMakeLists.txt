# SPDX-License-Identifier: Apache-2.0

#cmake_minimum_required(VERSION 3.20.0)

#message("CUSTOM MODULES ...")
# zephyr_custom_drivers/modules/CMakeLists.txt
message(STATUS "Processing custom_drivers module")
message(STATUS "Module directory: ${CMAKE_CURRENT_SOURCE_DIR}")

# Explicitly add the include directory
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/sensor/vl53l0x/inc)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/sensor/bno08x/inc)


if(CONFIG_CUSTOM_VL53L0X)
	add_subdirectory(sensor/vl53l0x)
endif()

if(CONFIG_CUSTOM_BNO08X)
	add_subdirectory(sensor/bno08x)
endif()
