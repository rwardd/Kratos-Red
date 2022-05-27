# SPDX-License-Identifier: Apache-2.0


set(CONF_FILE prj.conf)
#set(BOARD particle_argon)
set(DTC_OVERLAY_FILE argon.overlay)
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(kratos_red)

include_directories(./inc)
target_sources(app PRIVATE src/main.c inc/MAX30102.c inc/algorithm.c inc/heart_rate.c)
