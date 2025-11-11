#ifndef CONFIG_H
#define CONFIG_H

// ===== RETURN CODES =====
#define SUCCESS 0
#define FAILURE -1

// ===== HARDWARE PINS =====
// -- MPU9250 --
#define RGB_PIN 48
#define SDA_PIN 8
#define SCL_PIN 9
// -- EDF --
#define EDF_PIN 25
// -- SERVOS --
#define SERVO1_PIN 10
#define SERVO2_PIN 11
#define SERVO3_PIN 12
#define SERVO4_PIN 13
// -- RECEIVER --
// - CRSF -
#define CRSF_TX_PIN 17
#define CRSF_RX_PIN 18

// ==== CHANNELS ====
#define EDF_CHANNEL 0
#define SERVO1_CHANNEL 1
#define SERVO2_CHANNEL 2
#define SERVO3_CHANNEL 3
#define SERVO4_CHANNEL 4

// == MISC ==
#define BAUD_RATE 115200
#define CRSF_BAUD_RATE 420000
#define ALPHA 0.98f
#define DT 0.01f
#define I2C_FREQ 400000 // 400kHz sensor clock speed (MAX)
#define GX_BIAS 2.50f
#define GY_BIAS -4.65f
#define GZ_BIAS -1.13f
#define CRSF_UART_NUM UART_NUM_1
#define PWM_FREQ 50
#define PWM_RES_BITS 16
#define PWM_MAX_DUTY ((1 << PWM_RES_BITS) - 1)
#define SERVO_MAX_US 2000
#define SERVO_MIN_US 1000
#define USE_CSRF_INPUT 1

// ===== CONTROL LOOP TIMING =====
#define RATE_LOOP_HZ 250  // inner loop (gyro rate feedback)
#define ANGLE_LOOP_HZ 100 // outer loop (gyro angle feedback)

// ===== PHYSICAL LIMITS =====
#define MAX_ROLL_ANGLE 30.0f    // degrees
#define MAX_PITCH_ANGLE 30.0f
#define MAX_YAW_RATE 200.0f     // deg/s
#define MAX_SERVO_RATE 45.0f    // degrees deflection
#define MIN_THROTTLE 0.0f
#define MAX_THROTTLE 1.0f

// ===== PID GAINS (INITIAL) =====
// roll rate PID (inner loop)
#define ROLL_RATE_KP 0.0f
#define ROLL_RATE_KI 0.0f
#define ROLL_RATE_KD 0.0f
// roll angle PID (outer loop)
#define ROLL_ANGLE_KP 0.0f
#define ROLL_ANGLE_KI 0.0f
#define ROLL_ANGLE_KD 0.0f
// pitch rate PID
#define PITCH_RATE_KP 0.0f
#define PITCH_RATE_KI 0.0f
#define PITCH_RATE_KD 0.0f
// pitch angle PID
#define PITCH_ANGLE_KP 0.0f
#define PITCH_ANGLE_KI 0.0f
#define PITCH_ANGLE_KD 0.0f
// yaw rate PID
#define YAW_RATE_KP 0.0f
#define YAW_RATE_KI 0.0f
#define YAW_RATE_KD 0.0f

#endif // CONFIG_H