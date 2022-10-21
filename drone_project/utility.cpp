#include "utility.h"

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void led_setup()
{
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_put(LED_PIN, 1);
}

bool nrf_setup(bool role)
{
    // Let these addresses be used for the pair
    uint8_t address[][6] = {"1Node", "2Node"};
    // It is very helpful to think of an address as a path instead of as
    // an identifying device destination

    // to use different addresses on a pair of radios, we need a variable to
    // uniquely identify which address this radio will use to transmit
    bool radioNumber = role; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

    // initialize the transceiver on the SPI bus
    spi.begin(spi0, 2, 3, 4);
    if (!radio.begin(&spi))
    {
        printf("radio hardware is not responding!!\n");
        return false;
    }

    // Set the PA Level low to try preventing power supply related problems
    // because these examples are likely run with nodes in close proximity to
    // each other.
    radio.setPALevel(RF24_PA_LOW); // RF24_PA_MAX is default.

    // save on transmission time by setting the radio to only transmit the
    // number of bytes we need to transmit a float
    // radio.setPayloadSize(sizeof(control_payload)); // char[7] & uint8_t datatypes occupy 8 bytes
    radio.enableDynamicPayloads();
    // set the TX address of the RX node into the TX pipe
    radio.openWritingPipe(address[radioNumber]); // always uses pipe 0

    // set the RX address of the TX node into a RX pipe
    radio.openReadingPipe(1, address[!radioNumber]); // using pipe 1

    if (role){
        radio.stopListening(); // put radio in TX mode
    }
    else {
        radio.startListening();
    }
    return true;
} // setup()