#include "esc.h"

esc::esc(int pin)
{
    this->pin = pin;
    gpio_set_function(pin, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(pin);
    chan = pwm_gpio_to_channel(pin);
}

void esc::set_frequency(uint freq)
{
    frequency = freq;
    uint32_t clock = 125000000;
    uint32_t divider16 = clock / freq / 4096 +
                         (clock % (freq * 4096) != 0);
    if (divider16 / 16 == 0)
        divider16 = 16;
    wrap = clock * 16 / divider16 / freq - 1;
    pwm_set_clkdiv_int_frac(slice_num, divider16 / 16,
                            divider16 & 0xF);
    pwm_set_wrap(slice_num, wrap);
}

void esc::set_duty_cycle(int32_t duty)
{
    pwm_set_chan_level(slice_num, chan, wrap * duty / 100);
    pwm_set_enabled(slice_num, true);
}

void esc::set_level_ms(uint32_t milis)
{
    /* considering max wrap value at 100% duty cycle
     * we will derive the on time (in milisecond)
     */
    float time_period = 1000 * 1000 / frequency; // f=1/T
    // float on_time_percent = milis * 100 / time_period;
    // uint16_t level = (uint16_t)(wrap * on_time_percent / 100);

    uint16_t level = (uint16_t)(wrap * (milis * 100 / time_period) / 100);

    if (level > wrap)
    {
        pwm_set_chan_level(slice_num, chan, wrap);
    }
    pwm_set_chan_level(slice_num, chan, level);
    pwm_set_enabled(slice_num, true);
}
