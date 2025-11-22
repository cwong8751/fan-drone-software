#include "cf.h"
#include <math.h>

CF::CF(float alpha) : alpha(alpha), roll(0.0f), pitch(0.0f) {}

void CF::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) 
{
    // compute roll/pitch from accelerometer (tilt angles)
    float roll_acc = atan2(ay, az) * 180.0f / M_PI;
    float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;

    // integrate gyro to estimate new angles
    float roll_gyro = roll + gx * dt;
    float pitch_gyro = pitch + gy * dt;
    float yaw_gyro = yaw + gz * dt;

    // complementary filter fusion
    roll = alpha * roll_gyro + (1.0f - alpha) * roll_acc;
    pitch = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;

    if (mx != 0.0f || my != 0.0f || mz != 0.0f) 
    {
        // Compensate mag readings for current roll/pitch tilt
        float roll_rad  = roll * M_PI / 180.0f;
        float pitch_rad = pitch * M_PI / 180.0f;

        float mag_x = mx * cos(pitch_rad) + mz * sin(pitch_rad);
        float mag_y = mx * sin(roll_rad) * sin(pitch_rad)
                    + my * cos(roll_rad)
                    - mz * sin(roll_rad) * cos(pitch_rad);

        float yaw_mag = atan2(-mag_y, mag_x) * 180.0f / M_PI;

        // Fuse gyro yaw with magnetometer yaw
        yaw = alpha * yaw_gyro + (1.0f - alpha) * yaw_mag;
    } else {
        // No mag data available → rely only on gyro integration
        yaw = yaw_gyro;
    }

    if (yaw > 180.0f) yaw -= 360.0f;
    else if (yaw < -180.0f) yaw += 360.0f;
}

float CF::getRoll() const {
    return roll;
}

float CF::getPitch() const {
    return pitch;
}

float CF::getYaw() const {
    return yaw;
}