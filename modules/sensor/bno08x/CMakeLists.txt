# SPDX-License-Identifier: Apache-2.0

message("CUSTOM_BNO08X ...")
zephyr_library()
zephyr_library_sources(src/main.c)

# Explicitly add the include directory
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inc)

#zephyr_library_sources_ifdef(CONFIG_BNO086 src/main.c)

