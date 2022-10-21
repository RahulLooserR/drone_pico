add_library(mpu6050_driver INTERFACE)
target_sources(mpu6050_driver
    INTERFACE 
      ${CMAKE_CURRENT_LIST_DIR}/mpu6050_driver.cpp
)
target_include_directories(mpu6050_driver 
    INTERFACE 
      ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(mpu6050_driver 
    INTERFACE
      pico_stdlib
      hardware_spi 
      hardware_gpio
)
