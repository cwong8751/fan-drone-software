//#include "ekf.h"
#include "cf.h"
#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include <cmath>

// constants
#define RGB_pin 48
#define SDA_pin 8
#define SCL_pin 9
#define BAUD_RATE 115200
#define ALPHA 0.98f
#define DT 0.01f
#define GX_BIAS 2.50f
#define GY_BIAS -4.65f
#define GZ_BIAS -1.13f

// filtered angles
float roll = 0.0f;
float pitch = 0.0f;

// raw sensor values
float ax, ay, az;
float gx, gy, gz;

MPU9250 mpu;
CF cf(ALPHA);

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
    
    Serial.print("OK.\n");
    
    Serial.println("Initializing MPU9250 sensor...");

    // initialize MPU9250 
    if (!mpu.setup(0x68)) {
        Serial.println("FAILED.\n");
        setrgb(255, 0, 0); 
        while (1) delay(100);
    }
    
    Serial.print("OK.\n");

    Serial.println("Calibrating gyro, accel, and mag sensors...");

    // calibrate sensors (assumes device is stationary, optional in the future)
    mpu.calibrateAccelGyro();

    Serial.print("GYRO/ACCEL OK...");

    mpu.calibrateMag();
    delay(3000);

    Serial.print("OK.\n");
    
    setrgb(0, 255, 0);  

    Serial.println("gyro bias calculation...");

    float gx_bias = 0, gy_bias = 0, gz_bias = 0;
    for (int i = 0; i < 500; i++) {
        mpu.update_accel_gyro();
        gx_bias += mpu.getGyroX();
        gy_bias += mpu.getGyroY();
        gz_bias += mpu.getGyroZ();
        delay(2);
    }
    gx_bias /= 500; gy_bias /= 500; gz_bias /= 500;

    Serial.print("OK");

    Serial.println();
    Serial.print("Biases: ");
    Serial.print(gx_bias); Serial.print(", ");
    Serial.print(gy_bias); Serial.print(", ");
    Serial.println(gz_bias);

    // embed gyro biases to calculations
    // trial 1: 2.55, -4.69, -1.09
    // trial 2: 2.46, -4.62, -1.17
    // trial 3: 2.49, -4.65, -1.13
    // avg: gx bias of 2.50, gy bias of -4.65, gz bias of -1.13
}

void loop() {
    if (mpu.update()) {

        // extract raw gyro and accel data from MPU9250
        float gx = mpu.getGyroX() - GX_BIAS;
        float gy = mpu.getGyroY() - GY_BIAS;
        float gz = mpu.getGyroZ() - GZ_BIAS;

        float ax = mpu.getAccX();
        float ay = mpu.getAccY();
        float az = mpu.getAccZ();

        cf.update(gx, gy, gz, ax, ay, az, DT);

        Serial.print("roll:");
        Serial.print(cf.getRoll());
        Serial.print(",");
        Serial.print("pitch:");
        Serial.println(cf.getPitch());
        
        /*
        float roll = gx;
        float pitch = gy;
        float yaw = gz;

        Serial.print("roll:");
        Serial.print(roll);
        Serial.print(",");
        Serial.print("pitch:");
        Serial.print(pitch);
        Serial.print(",");
        Serial.print("yaw:");
        Serial.println(yaw);
        */

        delay(10);
    }
}
