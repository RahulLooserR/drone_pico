#include "joystick.hpp"

joystick::joystick()
{
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(x);
    adc_gpio_init(y);
    // adc_gpio_init(z);
}

void joystick::read(adc_value *adc_val)
{   
    // Select ADC input 0 (GPIO26)
    adc_select_input(input_x);
    adc_val->x =  adc_read();

    adc_select_input(input_y);
    adc_val->y = adc_read();

    // adc_select_input(input_z);
    // adc_val->z = adc_read();
    adc_val->z = 0;
}

void joystick::print_values(const adc_value *adc_val)
{
    std::cout << "x-axis value1: " << adc_val->x << std::endl;
    std::cout << "y-axis value: " << adc_val->y << std::endl;
    std::cout << "z-axis value: " << adc_val->z << std::endl;
}