cmake_minimum_required(VERSION 3.16)

#include(pico_sdk_import.cmake)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(text_example)

pico_sdk_init()

add_subdirectory(pico-ssd1306)

add_executable(text_example
        main.cpp)

target_link_libraries(text_example
        hardware_i2c
        pico_stdlib
        pico_ssd1306)


pico_enable_stdio_usb(text_example 1)
pico_enable_stdio_uart(text_example 0)

pico_add_extra_outputs(text_example)
