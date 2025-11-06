#include "rc.h"
#include "config.h"

void readRC(RC &rc)
{
    rc.throttle = pulseIn(RX_THROTTLE_PIN, HIGH, 25000); // timeout after 25ms
    rc.roll = pulseIn(RX_ROLL_PIN, HIGH, 25000);
    rc.pitch = pulseIn(RX_PITCH_PIN, HIGH, 25000);
    rc.yaw = pulseIn(RX_YAW_PIN, HIGH, 25000);

    // default values if no signal
    if (rc.throttle == 0) rc.throttle = 1000;
    if (rc.roll == 0) rc.roll = 1500;
    if (rc.pitch == 0) rc.pitch = 1500;
    if (rc.yaw == 0) rc.yaw = 1500;
}