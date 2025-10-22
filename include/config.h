#ifndef CONFIG_H
#define CONFIG_H

// ===== HARDWARE PINS =====
// -- MPU9250 --
#define SDA_PIN 8
#define SCL_PIN 9
#define I2C_FREQ 1000000 // 1MHz sensor clock speed

// -- LED --
#define RGB_pin 48

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