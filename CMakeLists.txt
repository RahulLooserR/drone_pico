cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_drone_lib 
  VERSION 1.0 
  DESCRIPTION "Raspberry Pi Pico NRF24L01P MPU6050 driver" 
  LANGUAGES C CXX
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_subdirectory(lib)
add_subdirectory(drone_project)