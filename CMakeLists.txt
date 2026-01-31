# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(blinky)

target_sources(app PRIVATE
    src/main.c
    src/modules/ringbuf/ringbuf.c
    src/modules/protocol/protocol.c
    src/modules/drivers/led_pwm.c
    src/modules/drivers/adc_ctrl.c
    src/modules/tx_buffer/tx_buffer.c
    src/modules/threads/tx_thread.c
)
