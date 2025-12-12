/*#ifndef PID_H
#define PID_H

#include "config.h"

struct PID {
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_error;
    float output_limit;
};

void pid_init(PID &pid, float kp, float ki, float kd, float limit);
float pid_update(PID &pid, float setpoint, float measurement, float dt);

struct AttitudeCommand {
    float roll_cmd;
    float pitch_cmd;
    float yaw_cmd;
};

AttitudeCommand compute_cascaded_pid(
    float rc_roll, float rc_pitch, float rc_yaw,
    float angle_roll, float angle_pitch, float angle_yaw,
    float gyro_x, float gyro_y, float gyro_z,
    float dt
);

#endif // PID_H
*/