#include "receiver_util.h"
#include "utility.h"

/* PID control variables */
// Desired roll rate -> drr
// Desired pitch rate -> dpr
float drr, dpr, dyr;
int throttle;
float pid_return[3];

float err_rr, err_pr, err_yr;
float prev_err_rr, prev_err_pr, prev_err_yr;
float prev_iterm_rr, prev_iterm_pr, prev_iterm_yr;

float ip_roll, ip_pitch, ip_yaw;

/* pid for roll */
const float p_constant_roll = 0.6;
const float i_constant_roll = 3.5;
const float d_constant_roll = 0.03;

/* pid for pitch */
const float p_constant_pitch = 0.6;
const float i_constant_pitch = 3.5;
const float d_constant_pitch = 0.03;

/* pid constant for yaw */
const float p_constant_yaw = 2;
const float i_constant_yaw = 12;
const float d_constant_yaw = 0;

static void PID_controller(float err, float p, float i, float d,
                           float prev_err, float prev_iterm);

static void reset_pid();

/* function for drone stablization
 * this will run on core 1 of pico
 * this function is for running motors
 * at desired rate
 */

void receiver_core1()
{
  float motor1, motor2, motor3, motor4;
  float avg_milis = (MAX_THROTTLE + MIN_THROTTLE) / 2;
  int count = 0;

  while (true)
  {
    if (multicore_fifo_rvalid())
    {
      if (multicore_fifo_pop_blocking())
      {
        if (control_payload.power_off)
          break;
        /* our desired rate is 75 degree/s
         * and input range from 1500-2000us
         * for motor to operate at min to max efficiency
         */

        mpu.read_calibrated();

        drr = 0.15 * (control_payload.pitch - avg_milis);
        dpr = 0.15 * (control_payload.roll - avg_milis);
        dyr = 0.15 * (control_payload.yaw - avg_milis);
        throttle = control_payload.throttle;

        /* Error roll rate -> err_rr */
        err_rr = drr - mpu.gyro_calibrated[0]; // roll rate from mpu sensor
        err_pr = dpr - mpu.gyro_calibrated[1];
        err_yr = dyr - mpu.gyro_calibrated[2];

        /* pid controller for roll */
        PID_controller(err_rr, p_constant_roll, i_constant_roll, d_constant_yaw,
                       prev_err_rr, prev_iterm_rr);
        ip_roll = pid_return[0];
        prev_err_rr = pid_return[1];
        prev_iterm_rr = pid_return[2];

        /* pid controller for pitch */
        PID_controller(err_pr, p_constant_pitch, i_constant_pitch, d_constant_pitch,
                       prev_err_pr, prev_iterm_pr);
        ip_pitch = pid_return[0];
        prev_err_pr = pid_return[1];
        prev_iterm_pr = pid_return[2];

        /* pid controller for yaw */
        PID_controller(err_yr, p_constant_yaw, i_constant_yaw, d_constant_yaw,
                       prev_err_yr, prev_iterm_yr);
        ip_yaw = pid_return[0];
        prev_err_yr = pid_return[1];
        prev_iterm_yr = pid_return[2];

        if (throttle > MAX_THROTTLE)
          throttle = MAX_THROTTLE;

        /* motors input */
        motor1 = 1.024 * (throttle - ip_roll - ip_pitch - ip_yaw);
        motor2 = 1.024 * (throttle - ip_roll + ip_pitch + ip_yaw);
        motor3 = 1.024 * (throttle + ip_roll + ip_pitch - ip_yaw);
        motor4 = 1.024 * (throttle + ip_roll - ip_pitch + ip_yaw);

        /* TODO */
        if (motor1 > MAX_THROTTLE)
          motor1 = MAX_THROTTLE;
        if (motor2 > MAX_THROTTLE)
          motor2 = MAX_THROTTLE;
        if (motor3 > MAX_THROTTLE)
          motor3 = MAX_THROTTLE;
        if (motor4 > MAX_THROTTLE)
          motor4 = MAX_THROTTLE;

        if (motor1 < MIN_THROTTLE + 20)
          motor1 = MIN_THROTTLE + 20;
        if (motor2 < MIN_THROTTLE + 20)
          motor2 = MIN_THROTTLE + 20;
        if (motor3 < MIN_THROTTLE + 20)
          motor3 = MIN_THROTTLE + 20;
        if (motor4 < MIN_THROTTLE + 20)
          motor4 = MIN_THROTTLE + 20;

        /* setting millis for all motors */
        bldc_FR.set_level_ms(motor1);
        bldc_BR.set_level_ms(motor2);
        bldc_BL.set_level_ms(motor3);
        bldc_FL.set_level_ms(motor4);

        count++;
        if (count == 251)
        {
          printf("\nFR : %f  ", motor1);
          printf("BR : %f  ", motor2);
          printf("BL : %f  ", motor3);
          printf("FL : %f\n", motor4);
          count = 0;
        }
      }
    }
  }
  bldc_FL.set_level_ms(ARM_BLDC);
  bldc_FR.set_level_ms(ARM_BLDC);
  bldc_BL.set_level_ms(ARM_BLDC);
  bldc_BR.set_level_ms(ARM_BLDC);
  sleep_ms(1000);
  return;
}

void PID_controller(float err, float p, float i, float d,
                    float prev_err, float prev_iterm)
{
  float pterm = p * err;
  float iterm = i * (err + prev_err) * 0.004 / 2; // 250hz loop, 1sec/0.004ms = 250
  float dterm = d * (err - prev_err) / 0.004;

  // i dont know why this is done
  if (iterm > 400)
  {
    iterm = 400;
  }
  else if (iterm < -400)
  {
    iterm = -400;
  }

  float pid_output = pterm + iterm + dterm;

  if (pid_output > 400)
  {
    pid_output = 400;
  }
  else if (pid_output < -400)
  {
    pid_output = -400;
  }

  pid_return[0] = pid_output;
  pid_return[1] = err;
  pid_return[2] = iterm;
}

void reset_pid()
{
  prev_err_rr = 0;
  prev_err_pr = 0;
  prev_err_yr = 0;

  prev_iterm_rr = 0;
  prev_iterm_pr = 0;
  prev_iterm_yr = 0;
}

void send_data_rx()
{
  uint8_t pipe;
  bool report = false;

  if (radio.available(&pipe))
  {                                                        // is there a payload? get the pipe number that recieved it
    radio.read(&control_payload, sizeof(control_payload)); // get incoming payload

    if (control_payload.send_ack)
    {
      // transmit response & save result to `report`
      radio.stopListening();                                          // put in TX mode
      radio.writeFast(&telemetry_payload, sizeof(telemetry_payload)); // load response to TX FIFO
      report = radio.txStandBy(300);                                  // keep retrying for 150 ms
      radio.startListening();                                         // put back in RX mode
    }

    // print summary of transactions, starting with details about incoming payload
    // printf("Received %d bytes on pipe %d:\n pitch: %d, roll: %d, throttle:%d, yaw: %d, is_ack: %d\n",
    //        radio.getPayloadSize(),
    //        pipe,
    //        control_payload.pitch, control_payload.roll, control_payload.throttle,
    //        control_payload.yaw, control_payload.send_ack);

    if (report)
    {
      // print outgoing payload and its counter
      // printf(" Sent: %f %f\n", telemetry_payload.latitude, telemetry_payload.longitude);
      telemetry_payload.latitude += 1.5;
      telemetry_payload.longitude += 2.5;
    }
    else if (!control_payload.send_ack)
    {
      // printf(" Response failed.\n"); // failed to send response
    }
  }
} // loop

void setup()
{
  stdio_init_all(); // init necessary IO for the RP2040
#ifdef SERIAL_COM
  while (!tud_cdc_connected())
  {
    sleep_ms(10);
  }
#endif

  telemetry_payload.latitude = 10.1;
  telemetry_payload.longitude = 20.1;

  MPU_6050 mpu(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, 400 * 1000);
  mpu.set_low_pass_filter(DLPF_CFG);
  mpu.set_sensitivity(GYRO_FS_SEL);
  mpu.calibrate();
  mpu.print();

  while (!nrf_setup(false))
  { // if radio.begin() failed
    // hold program in infinite attempts to initialize radio
  }

  led_setup();

  /* BLDC setup */
  bldc_FL.set_frequency(FREQUENCY);
  bldc_FR.set_frequency(FREQUENCY);
  bldc_BL.set_frequency(FREQUENCY);
  bldc_BR.set_frequency(FREQUENCY);
  printf("sleep for 1 sec\n");
  sleep_ms(1000);

  // bldc_FL.set_level_ms(ARM_BLDC);
  // bldc_FR.set_level_ms(ARM_BLDC);
  // bldc_BL.set_level_ms(ARM_BLDC);
  // bldc_BR.set_level_ms(ARM_BLDC);

  bldc_FL.set_level_ms(MAX_THROTTLE);
  bldc_FR.set_level_ms(MAX_THROTTLE);
  bldc_BL.set_level_ms(MAX_THROTTLE);
  bldc_BR.set_level_ms(MAX_THROTTLE);
  printf("sleep for 10 sec\n!!POWER ON SUPPLY!!!\n");
  sleep_ms(10000);
  bldc_FL.set_level_ms(MAX_THROTTLE - 30);
  bldc_FR.set_level_ms(MAX_THROTTLE - 30);
  bldc_BL.set_level_ms(MAX_THROTTLE - 30);
  bldc_BR.set_level_ms(MAX_THROTTLE - 30);

  printf("sleep for 4 sec\n");
  sleep_ms(4000);

  bldc_FL.set_level_ms((ARM_BLDC + MIN_THROTTLE) / 2);
  bldc_FR.set_level_ms((ARM_BLDC + MIN_THROTTLE) / 2);
  bldc_BL.set_level_ms((ARM_BLDC + MIN_THROTTLE) / 2);
  bldc_BR.set_level_ms((ARM_BLDC + MIN_THROTTLE) / 2);

  sleep_ms(2000);
}
