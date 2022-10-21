/* 
 * common utility used by 
 * both receiver and transmitter
 */

#ifndef UTILITY_H
#define UTILITY_H

#include "joystick.hpp"

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include <iostream>
#include "pico/multicore.h"

#include "pico/bootrom.h" // reset_usb_boot()
#include <RF24.h>         // RF24 radio object
#include "pins.h"
#include "mpu6050_driver.h"


#define _1MHZ 1000000
#define ARM_BLDC 900    
#define MIN_THROTTLE 1000     //1070
#define MAX_THROTTLE 1800   // 1970 is max
#define POWER_OFF_PIN 14 // this will turn on or off the drone

#define SERIAL_COM

typedef struct telemetry_data {
    float latitude;
    float longitude;
} telemetry_data_t;

typedef struct control_data {
    float roll;
    float pitch;
    int throttle;
    float yaw;
    bool send_ack;
    bool power_off;
} control_data_t;

extern control_data_t control_payload;
extern telemetry_data_t telemetry_payload;
extern RF24 radio;
extern MPU_6050 mpu;

long map(long x, long in_min, long in_max, long out_min, long out_max);

/*
 * pin setup
 */
void led_setup();

/*
 * device setup
 */
bool nrf_setup(bool role);



void display_telemetrey_data(telemetry_data_t);

#endif