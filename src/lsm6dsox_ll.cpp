#include "lsm6dsox_ll.h"
#include <Wire.h>

#define CTRL1_XL  0x10
#define CTRL2_G   0x11
#define CTRL3_C   0x12
#define OUTX_L_G  0x22

static inline void wr(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(LSM6_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

bool lsm6_ll_init()
{
    wr(CTRL3_C, 0x44);     // BDU + auto-increment
    wr(CTRL1_XL, 0xA0);    // 1.66 kHz accel, ±8g
    wr(CTRL2_G,  0xAC);    // 1.66 kHz gyro, ±2000 dps
    delay(10);
    return true;
}

bool lsm6_ll_read(IMURaw &o)
{
    Wire.beginTransmission(LSM6_ADDR);
    Wire.write(OUTX_L_G);
    if (Wire.endTransmission(false)) return false;

    if (Wire.requestFrom(LSM6_ADDR, 12) != 12) return false;

    uint8_t *p = (uint8_t *)&o;
    for (int i = 0; i < 12; i++) p[i] = Wire.read();
    return true;
}
