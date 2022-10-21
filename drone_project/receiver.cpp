#include "pins.h"
#include "receiver_util.h"
#include "utility.h"

// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);
control_data_t control_payload;
telemetry_data_t telemetry_payload;
MPU_6050 mpu(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, 400 * 1000);
esc bldc_FL(MOTOR_FRONT_LEFT_PIN);
esc bldc_FR(MOTOR_FRONT_RIGHT_PIN);
esc bldc_BR(MOTOR_BACK_RIGHT_PIN);
esc bldc_BL(MOTOR_BACK_LEFT_PIN);

int main()
{
  /* this is used to run the loop in specifc frequency
   * im considering 250 iterations per secreflist
   */
  
  setup();
 
  int counter = 0;
  absolute_time_t t1;
  absolute_time_t t2 = get_absolute_time();

  multicore_launch_core1(receiver_core1);

  while (true)
  {
    t1 = get_absolute_time();
    send_data_rx();
    multicore_fifo_push_blocking(1);

    // bldc_FL.set_level_ms(control_payload.throttle);
    // bldc_FR.set_level_ms(control_payload.throttle);
    // bldc_BR.set_level_ms(control_payload.throttle);
    // bldc_BL.set_level_ms(control_payload.throttle);

    counter += 1;
    if (absolute_time_diff_us(t2, get_absolute_time()) >= 1000000)
    {
      // printf("counter: %d\n", counter);
      counter = 0;
      // printf("current milis: %u\n", control_payload.throttle);
      // printf("Power off; %d\n", control_payload.power_off);
      // mpu.print();
      gpio_put(LED_PIN, !gpio_get_out_level(LED_PIN));
      t2 = get_absolute_time();
    }

    /* stricts loop to run 250 times / sec */
    while (absolute_time_diff_us(t1, get_absolute_time()) < 4000)
    {
    }
  }
  return 0; // we will never reach this
}