#ifndef RECEIVER_UTIL_H
#define RECEIVER_UTIL_H

#include <esc.h>
#include <tusb.h> // TinyUSB tud_cdc_connected()


#define FREQUENCY 500

#define MOTOR_FRONT_LEFT_PIN  13
#define MOTOR_FRONT_RIGHT_PIN 12
#define MOTOR_BACK_RIGHT_PIN  11
#define MOTOR_BACK_LEFT_PIN   10

extern esc bldc_FL; // Motor M1
extern esc bldc_FR; // Motor M2
extern esc bldc_BR; // Motor M3
extern esc bldc_BL; // Motor M4

void receiver_core1();
void send_data_rx();

void setup();

#endif