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

struct PWMChannel {
    volatile uint32_t risingEdge;
    volatile uint16_t pulseWidth;
    volatile bool newData;
};

extern PWMChannel pwm_channels;

extern RC rc;

void readRC(RC &rc);

void setupPWMInput();
void IRAM_ATTR throttleISR();
void IRAM_ATTR rollISR();
void IRAM_ATTR pitchISR();
void IRAM_ATTR yawISR();

#endif // RC_H