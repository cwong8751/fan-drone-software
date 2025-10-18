#include "cf.h"
#include <math.h>

CF::CF(float alpha) : alpha(alpha), roll(0.0f), pitch(0.0f) {}

void CF::update(float gx, float gy, float gz, float ax, float ay, float az, float dt) 
{
    // compute roll/pitch from accelerometer (tilt angles)
    float roll_acc = atan2(ay, az) * 180.0f / M_PI;
    float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;

    // integrate gyro to estimate new angles
    float roll_gyro = roll + gx * dt;
    float pitch_gyro = pitch + gy * dt;

    // complementary filter fusion
    roll = alpha * roll_gyro + (1.0f - alpha) * roll_acc;
    pitch = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
}

float CF::getRoll() const {
    return roll;
}

float CF::getPitch() const {
    return pitch;
}