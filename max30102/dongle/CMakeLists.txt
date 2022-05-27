# SPDX-License-Identifier: Apache-2.0

include_directories(./inc)

set(CONF_FILE prj.conf)
set(DTC_OVERLAY_FILE dtc_shell.overlay)
set(BOARD nrf52840dongle_nrf52840)
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(shell_example)


target_sources(app PRIVATE src/main.c inc/ahu_led.c inc/ahu_ble.c)

zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)