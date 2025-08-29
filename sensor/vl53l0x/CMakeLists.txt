# SPDX-License-Identifier: Apache-2.0

message("CUSTOM_VL53L0X Sensor ...")
zephyr_library()
zephyr_library_sources(src/custom_vl53l0x.c)

set(SELECT_API "_api" CACHE STRING "API Selection")

# Explicitly add the include directory
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inc)
#zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/api/core/inc)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/api/platform/inc)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/_api/core/inc)

#zephyr_library_sources_ifdef(CONFIG_VL53 src/main.c)
zephyr_library_sources(${CMAKE_CURRENT_SOURCE_DIR}/src/vl53l0x_tests.c)
zephyr_library_sources(${CMAKE_CURRENT_SOURCE_DIR}/src/${SELECT_API}/core/src/vl53l0x_api.c)
zephyr_library_sources(${CMAKE_CURRENT_SOURCE_DIR}/src/${SELECT_API}/core/src/vl53l0x_api_calibration.c)
zephyr_library_sources(${CMAKE_CURRENT_SOURCE_DIR}/src/${SELECT_API}/core/src/vl53l0x_api_core.c)
zephyr_library_sources(${CMAKE_CURRENT_SOURCE_DIR}/src/${SELECT_API}/core/src/vl53l0x_api_ranging.c)
zephyr_library_sources(${CMAKE_CURRENT_SOURCE_DIR}/src/${SELECT_API}/core/src/vl53l0x_api_strings.c)
zephyr_library_sources(${CMAKE_CURRENT_SOURCE_DIR}/src/api/platform/src/custom_vl53l0x_platform.c)		

if(NOT SELECT_API STREQUAL "_api")
	zephyr_library_sources(${CMAKE_CURRENT_SOURCE_DIR}/src/${SELECT_API}/platform/src/custom_vl53l0x_platform.c)		
endif()
