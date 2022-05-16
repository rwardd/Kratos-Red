

set(BOARD particle_argon)
set(CONF_FILE prj.conf)
set(DTC_OVERLAY_FILE argon.overlay)
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(argon_i2c)

include_directories(
                      ~/csse4011/repo/project/argon/lib/
                      )

target_sources(app PRIVATE
  src/main.c
  ~/csse4011/repo/project/argon/lib/argon_i2c.c
)