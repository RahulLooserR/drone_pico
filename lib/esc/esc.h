#ifndef ESC_H
#define ESC_H

#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "hardware/irq.h"

class esc
{
private:
    int pin;
    uint32_t frequency;
    uint32_t wrap;
    uint slice_num;
    uint chan;

public:
    esc(int pin);
    void set_frequency(uint freq);
    void set_duty_cycle(int32_t duty);
    void set_level_ms(uint32_t milis);
};

#endif