#include "rc.h"
#include "config.h"

PWMChannel channels[4];

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

void IRAM_ATTR throttleISR()
{
    if (digitalRead(RX_THROTTLE_PIN))
    {
        channels[0].risingEdge = micros();
    }
    else
    {  
        uint32_t width = micros() - channels[0].risingEdge;
        if (width > 800 && width < 2200)
        {
            channels[0].pulseWidth = width;
            channels[0].newData = true;
        }
    }
}

void IRAM_ATTR rollISR() {
    if (digitalRead(RX_ROLL_PIN) == HIGH) {
        channels[1].risingEdge = micros();
    } else {
        uint32_t width = micros() - channels[1].risingEdge;
        if (width > 800 && width < 2200) {
            channels[1].pulseWidth = width;
            channels[1].newData = true;
        }
    }
}

void IRAM_ATTR pitchISR() {
    if (digitalRead(RX_PITCH_PIN) == HIGH) {
        channels[2].risingEdge = micros();
    } else {
        uint32_t width = micros() - channels[2].risingEdge;
        if (width > 800 && width < 2200) {
            channels[2].pulseWidth = width;
            channels[2].newData = true;
        }
    }
}

void IRAM_ATTR yawISR() {
    if (digitalRead(RX_YAW_PIN) == HIGH) {
        channels[3].risingEdge = micros();
    } else {
        uint32_t width = micros() - channels[3].risingEdge;
        if (width > 800 && width < 2200) {
            channels[3].pulseWidth = width;
            channels[3].newData = true;
        }
    }
}

void setupPWMInput()
{
    pinMode(RX_THROTTLE_PIN, INPUT);
    pinMode(RX_ROLL_PIN, INPUT);
    pinMode(RX_PITCH_PIN, INPUT);
    pinMode(RX_YAW_PIN, INPUT);
    
    attachInterrupt(digitalPinToInterrupt(RX_THROTTLE_PIN), throttleISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RX_ROLL_PIN), rollISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RX_PITCH_PIN), pitchISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RX_YAW_PIN), yawISR, CHANGE);
}