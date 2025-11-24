#include "pid.h"
#include "config.h"

// ===== BASIC PID =====
void pid_init(PID &pid, float kp, float ki, float kd, float limit)
{
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
    pid.integral = 0.0f;
    pid.prev_error = 0.0f;
    pid.output_limit = limit;
}

float pid_update(PID &pid, float setpoint, float measurement, float dt)
{
    // calculate error
    float error = setpoint - measurement;

    // integral
    pid.integral += error * dt;

    // derivative
    float derivative = (error - pid.prev_error) / dt;
    pid.prev_error = error;

    // PID output
    float output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;

    return output;
}

// ===== CASCADED PID =====

// Outer (angle) PIDs
static PID roll_angle_pid;
static PID pitch_angle_pid;
static PID yaw_angle_pid;

// Inner (rate) PIDs
static PID roll_rate_pid;
static PID pitch_rate_pid;
static PID yaw_rate_pid;

static bool initialized = false;

AttitudeCommand compute_cascaded_pid(float rc_roll, float rc_pitch, float rc_yaw, float angle_roll, float angle_pitch, float angle_yaw, float gyro_x, float gyro_y, float gyro_z, float dt)
{
    if (!initialized)
    {
        // initialize PIDs with gains from config
        pid_init(roll_angle_pid, PID_ROLL_ANGLE_KP, PID_ROLL_ANGLE_KI, PID_ROLL_ANGLE_KD, PID_ROLL_ANGLE_LIMIT);
        pid_init(pitch_angle_pid, PID_PITCH_ANGLE_KP, PID_PITCH_ANGLE_KI, PID_PITCH_ANGLE_KD, PID_PITCH_ANGLE_LIMIT);
        pid_init(yaw_angle_pid, PID_YAW_ANGLE_KP, PID_YAW_ANGLE_KI, PID_YAW_ANGLE_KD, PID_YAW_ANGLE_LIMIT);

        pid_init(roll_rate_pid, PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD, PID_ROLL_RATE_LIMIT);
        pid_init(pitch_rate_pid, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD, PID_PITCH_RATE_LIMIT);
        pid_init(yaw_rate_pid, PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD, PID_YAW_RATE_LIMIT);

        initialized = true;
    }

    // Outer loop: angle PIDs to get desired rates
    float roll_rate_setpoint = pid_update(roll_angle_pid, rc_roll * MAX_ANGLE_DEG, angle_roll, dt);
    float pitch_rate_setpoint = pid_update(pitch_angle_pid, rc_pitch * MAX_ANGLE_DEG, angle_pitch, dt);
    float yaw_rate_setpoint = pid_update(yaw_angle_pid, rc_yaw * MAX_ANGLE_DEG, angle_yaw, dt);

    // Inner loop: rate PIDs to get control commands
    float roll_cmd = pid_update(roll_rate_pid, roll_rate_setpoint, gyro_x, dt);
    float pitch_cmd = pid_update(pitch_rate_pid, pitch_rate_setpoint, gyro_y, dt);
    float yaw_cmd = pid_update(yaw_rate_pid, yaw_rate_setpoint, gyro_z, dt);

    AttitudeCommand cmd;
    cmd.roll_cmd = roll_cmd;
    cmd.pitch_cmd = pitch_cmd;
    cmd.yaw_cmd = yaw_cmd;

    return cmd;
}