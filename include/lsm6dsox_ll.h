#pragma once
#include <Arduino.h>

#define LSM6_ADDR 0x6B

struct IMURaw {
    int16_t gx, gy, gz;
    int16_t ax, ay, az;
};

bool lsm6_ll_init();
bool lsm6_ll_read(IMURaw &imu);
