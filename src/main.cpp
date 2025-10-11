#include "ekf.h"
#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include <BasicLinearAlgebra.h>
#include <cmath>

#define RGB_pin 48
#define SDA_pin 8
#define SCL_pin 9
#define BAUD_RATE 115200

MPU9250 mpu;
EKF ekf(0.01f); // 100 Hz sample rate

BLA::Matrix<3> gyro;
BLA::Matrix<3> accel;
BLA::Matrix<3> mag;

// helper function for LEDs indicating FC status 
void setrgb(uint8_t red, uint8_t green, uint8_t blue){
  neopixelWrite(RGB_pin, red, green, blue);
}

void setup() {
    Serial.begin(BAUD_RATE); // start serial comm at baud rate 115200 for USB

    Serial.println("Initializing peripherals...\n\n");

    Serial.println("Initializing I2C0 bus interface...");

    // intilialize first i2c bus to 8 (SDA) and 9 (SCL)  
    Wire.begin(SDA_pin, SCL_pin);
    Wire.setClock(400000); // 400kHz fast mode with MPU9250
    
    Serial.println("OK.\n");
    
    Serial.println("Initializing MPU9250 sensor...");

    // initialize MPU9250 
    if (!mpu.setup(0x68)) {
        Serial.println("FAILED.\n");
        setrgb(255, 0, 0); 
        while (1) delay(100);
    }
    
    Serial.println("OK.\n");

    Serial.println("Calibrating gyro, accel, and mag sensors...");

    // calibrate sensors (assumes device is stationary, optional in the future)
    mpu.calibrateAccelGyro();
    mpu.calibrateMag();

    Serial.println("OK.\n");
    
    setrgb(0, 255, 0);  
}

void loop() {
    if (mpu.update()) {

        // extract raw gyro, accel, and mag data from mpu 
        gyro  = {  mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ() };
        accel = {  mpu.getAccX(),  mpu.getAccY(),  mpu.getAccZ()  };
        mag   = {  mpu.getMagX(),  mpu.getMagY(),  mpu.getMagZ()  };

        // EKF algorithm steps
        ekf.predict(gyro);
        //ekf.update(accel, mag);
        //ekf.normalizeQuaternion();
    }
}
