add_executable(receiver
receiver.cpp
utility.cpp
receiver_util.cpp)

target_link_libraries(receiver 
    PRIVATE 
      RF24 
      mpu6050_driver
      joystick
      pico_stdlib
      pico_ssd1306
      pico_multicore
      esc
)

pico_enable_stdio_usb(receiver 1)
pico_enable_stdio_uart(receiver 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(receiver)

add_executable(transmitter
transmitter.cpp
utility.cpp
transmitter_util.cpp)

target_link_libraries(transmitter 
    PRIVATE 
      RF24 
      mpu6050_driver
      joystick
      pico_stdlib
      pico_ssd1306
      pico_multicore
      
)

pico_enable_stdio_usb(transmitter 1)
pico_enable_stdio_uart(transmitter 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(transmitter)