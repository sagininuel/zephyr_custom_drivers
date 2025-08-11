# SPDX-License-Identifier: Apache-2.0

message("CUSTOM_BNO08X ...")
zephyr_library()
zephyr_library_sources(src/main.c)

#zephyr_library_sources_ifdef(CONFIG_BNO086 src/main.c)

