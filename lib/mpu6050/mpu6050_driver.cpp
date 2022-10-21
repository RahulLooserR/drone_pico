#include "mpu6050_driver.h"
MPU_6050::MPU_6050(int sda, int scl, int freq, i2c_inst_t *interf)
{
    interface = interf;
    i2c_init(interface, freq);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    reset();
}

void MPU_6050::reset()
{
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(interface, i2c_addr, buf, 2, false);
}

void MPU_6050::read_gyro_raw()
{
    i2c_write_blocking(interface, i2c_addr, &gyro_data_reg, 1, true);
    i2c_read_blocking(interface, i2c_addr, buffer, 6, false); // False - finished with bus

    for (int i = 0; i < 3; i++)
    {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
        ;
    }
}

void MPU_6050::calibrate()
{
    int64_t avg_gyro[3] = {0};
    for (int i = 0; i < 2000; i++)
    {
        read_gyro_raw();
        for (int j = 0; j < 3; j++)
        {
            avg_gyro[j] += gyro[j];
        }
        sleep_ms(1);
    }

    for (int i = 0; i < 3; i++)
    {
        avg_gyro[i] /= 2000;
        error_gyro[i] = (int16_t)avg_gyro[i];
    }
    // printf("calibrated x: %d\n", error_gyro[0]);
    // printf("calibrated y: %d\n", error_gyro[1]);
    // printf("calibrated z: %d\n", error_gyro[2]);
}

void MPU_6050::read_calibrated()
{
    read_gyro_raw();
    for (int i = 0; i < 3; i++)
    {
        gyro[i] = gyro[i] - error_gyro[i];
        gyro_calibrated[i] = (float) gyro[i] / lsb;
    }
}

void MPU_6050::print()
{
    printf("Roll rate X:  %d d/sec ", error_gyro[0]);
    printf("Pitch rate Y:  %d d/sec ", error_gyro[1]);
    printf("Yaw rate Z:  %d d/sec\n", error_gyro[2]);

    printf("c_Roll rate X: %f d/sec ", gyro_calibrated[0]);
    printf("c_Pitch rate Y: %f d/sec ", gyro_calibrated[1]);
    printf("c_Yaw rate Z: %f d/sec\n\n", gyro_calibrated[2]);
}

void MPU_6050::set_low_pass_filter(uint8_t value)
{   
    uint8_t  buf[2] = {config_reg, value};
    i2c_write_blocking(interface, i2c_addr, buf, 2, false);
    reset();
}

void MPU_6050::set_sensitivity(uint8_t value)
{
    uint8_t  buf[2] = {sensitivity_reg, value};
    i2c_write_blocking(interface, i2c_addr, buf, 2, false);
    reset();
}