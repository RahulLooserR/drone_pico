#ifndef NRF24_DRIVER_H
#define NRF24_DRIVER_H
#include "nrf24_pin_config.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include <stdio.h>
#define SPI_BAUDRATE 1000000

typedef struct spi_pin_config_
{
    spi_inst_t *hspi;
    uint16_t sck;
    uint16_t tx;
    uint16_t rx;
    uint16_t csn;
    uint16_t ce;
} spi_pin_config_t;

/* all power setting are in negative */
typedef enum RF_PWR_ {
    _18DBM = (0x00 << 1),
    _12DBM = (0x01 << 1),
    _6DBM = (0x02 << 1),
    _0DBM = (0x03 << 1)
} rf_pwr_t;

typedef struct spi_config_
{
    // retries
    // setPA level
    // data rate
    // crc length
    // dynamic payload
    // payload size
    // reeset status
    // channel
    //flush tx rx
    uint8_t temp;

} spi_config_t;


class NRF24L01
{
private:
    uint16_t csn;
    uint16_t ce;
    spi_inst_t *hspi;

private:
    void csn_low() { gpio_put(csn, 0); }
    void csn_high() { gpio_put(csn, 1); }
    void ce_low() { gpio_put(ce, 0); }
    void ce_high() { gpio_put(ce, 1); }

public:
    NRF24L01(spi_pin_config_t *spi_pin_conf);
    ~NRF24L01();

public:
    uint8_t read_reg(uint8_t reg);
    void read_reg_bytes(uint8_t reg, uint8_t *buf, uint8_t size);
    void write_reg(uint8_t reg, uint8_t data);
    void write_reg_bytes(uint8_t reg, uint8_t *buf, uint8_t size = 1);
    void flush_rx();
    void flush_tx();
    void configure(spi_config_t *config);
    void open_writing_pipe(uint8_t *addr, uint8_t size);
    bool send_msg(void *buf, uint8_t size, int64_t timeout = 100); // 20 ms timeout

    void open_reading_pipe(uint8_t pipeno, uint8_t *addr, uint8_t size = 5);
    void start_listening();
    bool any();
    bool recv_msg(void *buf, uint8_t size = 32);

/* all debug functions here */
public:
    void print_value_one_byte(char *reg_name, uint8_t reg);
    void print_value(char *reg_name, uint8_t reg, uint8_t size);
    void print_reg_values();
};

#endif