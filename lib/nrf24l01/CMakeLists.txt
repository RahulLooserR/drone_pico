add_library(nrf24_driver INTERFACE)
target_sources(nrf24_driver
    INTERFACE 
      ${CMAKE_CURRENT_LIST_DIR}/nrf24_driver.cpp
)
target_include_directories(nrf24_driver 
    INTERFACE 
      ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(nrf24_driver 
    INTERFACE
      pico_stdlib
      hardware_spi 
      hardware_gpio
)
