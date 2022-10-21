#ifndef PINS_H
#define PINS_H

#define LED_PIN PICO_DEFAULT_LED_PIN

/* spi pins for NRF24
 * spi0(2, 3, 4)
 */
#define CE_PIN 6
#define CSN_PIN 5

/* i2c pins for oled screen ssd1306 */
#define OLED_SDA_PIN 20
#define OLED_SCL_PIN 21
#define OLED_PORT i2c0
#define SCREEN_ADDR 0x3C

/* ADC pins for
 * roll -> side by side movement
 * pitch -> forward/backward movement
 * throttle -> up/down
 * yaw -> rotation cw/ccw
 */
#define ROLL_PIN 26
#define PITCH_PIN 27
#define THROTTLE_PIN 28
#define YAW_CW_PIN 10
#define YAW_CCW_PIN 11
#define CW 1
#define CCW 0

/*
 * mpu6050 accelerometer pins
 */

/* 
 * GPS (Neo6m) pins
 */


#endif