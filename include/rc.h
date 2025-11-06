#ifndef RC_H
#define RC_H

#include <Arduino.h>

struct RC
{
    uint16_t throttle; 
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
};

extern RC rc;

void readRC(RC &rc);

#endif // RC_H