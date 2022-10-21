#ifndef TRANSMITTER_UTIL_H
#define TRANSMITTER_UTIL_H

#include "utility.h"
#include "../../lib/pico-ssd1306/ssd1306.h"
#include "../../lib/pico-ssd1306/textRenderer/TextRenderer.h"

extern uint64_t ping;
extern pico_ssd1306::SSD1306 display;
const int UP_PIN = 0;
const int DOWN_PIN = 1;

void core1_function();
void oled_setup();  // run on core 1
void pin_setup_tx();
void read_input(control_data_t *);
void print_input_value(const control_data_t *control_payload);
bool send_data_tx();

#endif