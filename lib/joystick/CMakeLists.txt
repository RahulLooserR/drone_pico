add_library(joystick INTERFACE)
target_sources(joystick
    INTERFACE 
      ${CMAKE_CURRENT_LIST_DIR}/joystick.cpp
)
target_include_directories(joystick 
    INTERFACE 
      ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(joystick 
    INTERFACE
      pico_stdlib
      hardware_spi 
      hardware_gpio
      hardware_adc
)
