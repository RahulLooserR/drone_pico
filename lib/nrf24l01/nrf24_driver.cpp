#include "nrf24_driver.h"

NRF24L01::NRF24L01(spi_pin_config_t *spi_pin_conf)
{
    this->hspi = spi_pin_conf->hspi;
    this->csn = spi_pin_conf->csn;
    this->ce = spi_pin_conf->ce;

    gpio_set_function(spi_pin_conf->sck, GPIO_FUNC_SPI);
    gpio_set_function(spi_pin_conf->tx, GPIO_FUNC_SPI);
    gpio_set_function(spi_pin_conf->rx, GPIO_FUNC_SPI);
    spi_init(this->hspi, SPI_BAUDRATE);

    gpio_init(csn);
    gpio_init(ce);

    gpio_set_dir(csn, 1);
    gpio_set_dir(ce, 1);

    ce_low();
    csn_high();
}

uint8_t NRF24L01::read_reg(uint8_t reg)
{
    uint8_t tx_buf[2] = {reg, CMD_NOP}; // NOP
    uint8_t rx_buf[2];
    csn_low();
    spi_write_read_blocking(hspi, tx_buf, rx_buf, 2);
    csn_high();
    return rx_buf[1];
}

void NRF24L01::read_reg_bytes(uint8_t reg, uint8_t *buf, uint8_t size)
{
    uint8_t tx_buf[size + 1] = {reg}; 
    uint8_t rx_buf[size + 1];
    for(uint8_t i = 1; i < size+1; i++) {
        tx_buf[i] = CMD_NOP;
    }
    csn_low();
    spi_write_read_blocking(hspi, tx_buf, rx_buf, size + 1);
    csn_high();

    for(uint8_t i = 1; i < size+1; i++) {
        buf[i - 1] = rx_buf[i];
    }
}

void NRF24L01::write_reg(uint8_t reg, uint8_t data)
{
    write_reg_bytes(reg, &data, 1);
}

void NRF24L01::write_reg_bytes(uint8_t reg, uint8_t *buf, uint8_t size)
{
    uint8_t tx_buf[size + 1]; 
    uint8_t rx_buf[size + 1];
    reg = ((MASK_REGISTER & reg) | CMD_W_REGISTER);
    tx_buf[0] = reg;
    /* copying rest of the data */
    for(uint8_t i = 1; i < size + 1; i++) {
        tx_buf[i] = buf[i - 1];
    }
    csn_low();
    spi_write_read_blocking(hspi, tx_buf, rx_buf, size + 1);
    csn_high();
}
void NRF24L01::flush_rx()
{   
    uint8_t cmd = CMD_FLUSH_RX;
    csn_low();
    sleep_us(2);
    spi_write_blocking(hspi, &cmd, 1);
    sleep_us(2);
    csn_high();
}

void NRF24L01::flush_tx()
{
    uint8_t cmd = CMD_FLUSH_TX;
    csn_low();
    sleep_us(2);
    spi_write_blocking(hspi, &cmd, 1);
    sleep_us(2);
    csn_high();
}


void NRF24L01::configure(spi_config_t *config)
{
    uint8_t result = 0;
    /* power up bit */
    result = read_reg(REG_CONFIG);
    result = result | (1 << BIT_PWR_UP);
    //ce_low();
    write_reg(REG_CONFIG, result);
    ce_low();
    sleep_ms(150);
    

    /* setup retries, buf = 0x00*/
    write_reg(REG_SETUP_RETR, 0x00);

    /* setup PA level 
       default only
    */

   /* leave crc default */

   /* disable dynamic payload, bydefault disabled */

    /* set payload size = 32 */

    /* disable auto ack */
    write_reg(REG_EN_AA, 0x00);

    /* keeping channel default */

    
    flush_rx();
    flush_tx();

}

void NRF24L01::open_writing_pipe(uint8_t *addr, uint8_t size)
{
    write_reg_bytes(REG_RX_ADDR_P0, addr, size);
    write_reg_bytes(REG_TX_ADDR, addr, size);
    
    /*set payload size to 32 */
    write_reg(REG_RX_PW_P0, 32);
}

bool NRF24L01::send_msg(void *buf, uint8_t size, int64_t timeout)
{   
    flush_tx();
    uint8_t result = 0;
    volatile uint8_t status;
    uint64_t start_time = get_absolute_time()._private_us_since_boot;
    timeout = timeout * 1000;

    /* reset status register */
    uint8_t data = read_reg(REG_STATUS);
    data = data | (1 << BIT_RX_DR) | (1 << BIT_TX_DS) | (1 << BIT_MAX_RT);
    write_reg(REG_STATUS, data);

    /* tx mode */
    write_reg(REG_CONFIG, (read_reg(REG_CONFIG) & ~(1 << BIT_PRIM_RX)));

    uint8_t cmd = CMD_W_TX_PAYLOAD;
    csn_low();
    ce_high();
    spi_write_blocking(hspi, &cmd, 1);
    spi_write_blocking(hspi, (uint8_t *)buf, size);

    csn_high();
    sleep_ms(15);
    ce_low();

    while(get_absolute_time()._private_us_since_boot - start_time <= timeout) {
        status = read_reg(REG_STATUS);
        if  (status & (1 << BIT_TX_DS)){
            return true;
        }
    }
    
    return false;
}

void NRF24L01::open_reading_pipe(uint8_t pipeno, uint8_t *addr, uint8_t size)
{   
    char buf[20];
    /* by default seelcting pipe 1 */
    write_reg_bytes(REG_RX_ADDR_P0, addr, size);
    write_reg_bytes(REG_RX_ADDR_P1, addr, size);
    /* writing payload size 32 */
    write_reg(REG_RX_PW_P0, 32);
    write_reg(REG_EN_RXADDR, read_reg(REG_EN_RXADDR) | (1 << 1));

    sprintf(buf, "REG_RX_ADDR_P0");
    print_value(buf, REG_RX_ADDR_P0, 5);

    sprintf(buf, "REG_RX_ADDR_P1");
    print_value(buf, REG_RX_ADDR_P1, 5);

    sprintf(buf, "REG_RX_ADDR_P0");
    print_value(buf, REG_RX_ADDR_P0, 5);

    sprintf(buf, "REG_EN_RXADDR");
    print_value_one_byte(buf, REG_EN_RXADDR);
}

/* this is tx mode */
void NRF24L01::start_listening()
{   
    // uint8_t config = read_reg(REG_CONFIG);
    // config = config | (1 << BIT_PRIM_RX);
    write_reg(REG_CONFIG, read_reg(REG_CONFIG) | (1 << BIT_PRIM_RX));
    write_reg(REG_STATUS, (1 << BIT_RX_DR) | (1 << BIT_TX_DS) | (1 << BIT_MAX_RT));
    ce_high();
    sleep_ms(130);
    flush_rx();
    flush_tx();
}

bool NRF24L01::any()
{
    // printf("%d\n", read_reg(REG_STATUS));
    //if (read_reg(REG_STATUS) & (1 << BIT_RX_DR)) {
    if (read_reg(REG_FIFO_STATUS) & (1 << BIT_RX_EMPTY) == 0) {
        printf("data incoming:");
        return true;
        /* implement functionality to get pipe number */
    }
    return false;
}

bool NRF24L01::recv_msg(void *buf, uint8_t size)
{
    bool fifo_status;
    uint8_t cmd = CMD_R_RX_PAYLOAD;
    csn_low();
    spi_write_blocking(hspi, &cmd, 1);
    spi_read_blocking(hspi, CMD_NOP, (uint8_t *)buf, 32);
    csn_high();

    /* read fifo status */
    fifo_status = read_reg(REG_FIFO_STATUS) & (1 << BIT_RX_EMPTY);
    flush_rx();

    return fifo_status;
}

/* debug functions here */
void NRF24L01::print_value_one_byte(char *reg_name, uint8_t reg)
{
    uint8_t result = read_reg(reg);
    printf("\n%-20s: 0x%02X   ", reg_name, result);
    for(int8_t i = 7; i >= 0; i--) {
        printf((result & (1 << i)) ? "1 ":"0 ");
    }
}
void NRF24L01::print_value(char *reg_name, uint8_t reg, uint8_t size)
{
    uint8_t buf[size];
    read_reg_bytes(reg, buf, size);
    printf("\n%-20s: ", reg_name);
    for(uint8_t i = 0; i < size; i++){
        printf("%02X ", buf[i]);
    }
}
void NRF24L01::print_reg_values()
{
    char buf[100];
    sprintf(buf, "\n********************** 8 BIT reg info ***********************\n");
    printf("%s", buf);

    sprintf(buf, "REG_CONFIG");
    print_value_one_byte(buf, REG_CONFIG);

    sprintf(buf, "REG_EN_AA");
    print_value_one_byte(buf, REG_EN_AA);

    sprintf(buf, "REG_EN_RXADDR");
    print_value_one_byte(buf, REG_EN_RXADDR);

    sprintf(buf, "REG_SETUP_AW");
    print_value_one_byte(buf, REG_SETUP_AW);

    sprintf(buf, "REG_SETUP_RETR");
    print_value_one_byte(buf, REG_SETUP_RETR);

    sprintf(buf, "REG_RF_CH");
    print_value_one_byte(buf, REG_RF_CH);

    sprintf(buf, "REG_RF_SETUP");
    print_value_one_byte(buf, REG_RF_SETUP);

    sprintf(buf, "REG_STATUS");
    print_value_one_byte(buf, REG_STATUS);

    sprintf(buf, "REG_OBSERVE_TX");
    print_value_one_byte(buf, REG_OBSERVE_TX);

    sprintf(buf, "REG_CD");
    print_value_one_byte(buf, REG_CD);

    sprintf(buf, "REG_RX_PW_P0");
    print_value_one_byte(buf, REG_RX_PW_P0);

    sprintf(buf, "REG_RX_PW_P1");
    print_value_one_byte(buf, REG_RX_PW_P1);

    sprintf(buf, "REG_FIFO_STATUS");
    print_value_one_byte(buf, REG_FIFO_STATUS);

    sprintf(buf, "REG_DYNPD");
    print_value_one_byte(buf, REG_DYNPD);

    sprintf(buf, "REG_FEATURE");
    print_value_one_byte(buf, REG_FEATURE);

    sprintf(buf, "\n********************* other reg info ***********************\n");
    printf("%s", buf);

    sprintf(buf, "REG_TX_ADDR");
    print_value(buf, REG_TX_ADDR, 5);

    sprintf(buf, "REG_RX_ADDR_P0");
    print_value(buf, REG_RX_ADDR_P0, 5);

    sprintf(buf, "REG_RX_ADDR_P1");
    print_value(buf, REG_RX_ADDR_P1, 5);

}