# SPDX-License-Identifier: Apache-2.0


set(CONF_FILE prj.conf)
#set(BOARD particle_argon)
set(DTC_OVERLAY_FILE argon.overlay)
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(kratos_red)

include_directories(./inc
                    ../../particle_argon/lib)
target_sources(app PRIVATE 
    src/main.c 
    inc/MAX30102.c 
    inc/algorithm.c 
    inc/heart_rate.c 
    src/bpm_spo2.c 
    src/argon_ble.c
    ../../particle_argon/lib/argon_i2c.c)

zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)