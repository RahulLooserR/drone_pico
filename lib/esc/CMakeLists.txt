add_library(esc INTERFACE)
target_sources(esc
    INTERFACE 
      ${CMAKE_CURRENT_LIST_DIR}/esc.cpp
)
target_include_directories(esc 
    INTERFACE 
      ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(esc 
    INTERFACE
      pico_stdlib
      hardware_gpio
      hardware_pwm
      hardware_irq
)
