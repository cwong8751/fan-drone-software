#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>

// flight state structure (shared b/n our two cores)
struct FlightState 
{
    // orientation (using library sensor fusion)
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    // gyro rates (deg/s)
    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;

    // accelerometer (g's)
    float acc_x = 0.0f;
    float acc_y = 0.0f;
    float acc_z = 0.0f;

    // magnetometer (mG)
    float mag_x = 0.0f;
    float mag_y = 0.0f;
    float mag_z = 0.0f;

    // timestamp
    uint32_t timestamp_ms = 0;

    // data ready flag
    volatile bool data_ready = false;
};
extern FlightState state;

void motor_init();
void setup_pwm_timer();
void setup_pwm_config();
void motor_update_from_crsf();
int motor_arm();
void write_us(uint8_t channel, uint16_t usec);
#endif // CONTROL_H