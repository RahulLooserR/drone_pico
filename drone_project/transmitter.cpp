
#include "utility.h"
#include "transmitter_util.h"
#include "pins.h"


#include <tusb.h> // TinyUSB tud_cdc_connected()

control_data_t control_payload;
telemetry_data_t telemetry_payload;
uint64_t ping;

// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);

int main()
{
    stdio_init_all(); // init necessary IO for the RP2040
    led_setup();
    pin_setup_tx();
    control_payload.pitch = 10;
    control_payload.roll = 20;
    control_payload.throttle = ARM_BLDC;
    control_payload.yaw = 0;
    control_payload.send_ack = false;
    
    multicore_launch_core1(core1_function);

// wait here until the CDC ACM (serial port emulation) is connected
#ifdef SERIAL_COM
    while (!tud_cdc_connected())
    {
        sleep_ms(10);
    }
#endif

    sleep_ms(1000);
    while (!nrf_setup(true)) // true for tx role
    { // if radio.begin() failed
      // hold program in infinite attempts to initialize radio
    }
    while (true)
    {   
        read_input(&control_payload);
        if (send_data_tx())
        {
            multicore_fifo_push_blocking(1);
        }
    }
    return 0; // we will never reach this
}