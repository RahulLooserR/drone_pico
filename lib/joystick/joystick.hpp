#ifndef FUNC_H
#define FUNC_H
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include <iostream>


typedef struct adc_value_{
    uint16_t x;
    uint16_t y;
    uint16_t z;
} adc_value;

/* jostick1 configuration
   adc0 is used for left-right movement
   ADC0 -> x-axis (GP26, pin_no: 31)

   adc1 is used for forward-backward movement
   ADC1 -> y-axis (GP27, pin_no: 32)

   Joystick2 configuration
   adc2 is used for up-down movement
   ADC2 -> z-axis (GP28, pin_no: 34)
 */
class joystick{
    private:
        uint x = 26; /* select input as 0 */
        uint input_x = 0;
        uint y = 27; /* select input as 1 */
        uint input_y = 1;
        uint z = 28; /* select input as 2 */
        uint input_z = 2;
    
    public:
        joystick();
        void read(adc_value *adc_val);
        void print_values(const adc_value *adc_val);
};

#endif