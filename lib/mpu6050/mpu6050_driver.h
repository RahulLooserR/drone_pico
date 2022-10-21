#ifndef _MPU_6050_H
#define _MPU_6050_H

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include <stdio.h>

#define MPU6050_I2C    0x68

#define CONFIG_REG     0x1A     // default value 0x00
// vlaue to be written on config reg
#define DLPF_CFG       0x05     // default value 0x00

#define SESITIVITY_REG 0x1B     // default value 0x00
/*
 * value to be written on  sensitivity reg
 * for value 0x08  we are going to set 
 * +- 500 degree per secreflis
 * or 65.5 LSB
 */
#define GYRO_FS_SEL    0x08
#define LSB 65.5

class MPU_6050 {
    private:
        uint8_t i2c_addr = 0x68;
        uint8_t config_reg = 0x1A;
        uint8_t sensitivity_reg = 0x1B;

        uint8_t gyro_data_reg = 0x43;
        uint8_t accl_data_reg = 0x3B;
        uint8_t temp_data_reg = 0x41;
        int sda, scl;
        uint8_t buffer[6];

        int16_t error_gyro[3] = {-90, -357, 69};
        i2c_inst_t* interface;
        float lsb = 65.5; // for low pass filter DLPF_CFG = 0x05
    
    public:
        // int16_t acceleration[3], gyro[3], temp;
        int16_t gyro[3];
        float gyro_calibrated[3];
    
    public:
        MPU_6050(int sda, int scl, int freq, i2c_inst_t* interf=i2c_default);
        void set_low_pass_filter(uint8_t value);
        void set_sensitivity(uint8_t value);
        bool set_power(uint8_t value);

        void read_gyro_raw();
        void calibrate();
        void reset();
        void read_calibrated();
        void print();
};



#endif