#include "cf.h"
#include "metric.h"
#include "config.h"
#include "control.h"
#include "crsf_rc.h"
#include "driver/ledc.h"
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>

// sensor libraries
#include "Adafruit_LSM6DSOX.h"
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <MS5x.h>
#include "lsm6dsox_ll.h"

// ===== OPTION 1: Use Adafruit library (RECOMMENDED) =====
#include <Adafruit_AHRS.h>
Adafruit_Madgwick mad_filter;

// ===== OPTION 2: Use custom implementation =====
#include "madgwick_filter.h"
// MadgwickFilter mad_filter(450, 0.25f);

#include "sensor_config.h"  // Axis remapping utilities
#include "sensor_calibration.h"

SensorCalibrator calibrator;
Performance perf;
FlightState state;
CF cf(ALPHA);
int channels[] = {EDF_CHANNEL, SERVO1_CHANNEL, SERVO2_CHANNEL, SERVO3_CHANNEL, SERVO4_CHANNEL};
int pins[] = {EDF_PIN, SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN};
constexpr float DEG2RAD = 0.01745329252f;
constexpr float GYRO_SCALE =
    (500.0f / 32768.0f) * DEG2RAD;
constexpr float ACC_SCALE  = 8.0f / 32768.0f;

// sensor objects
MS5x barometer(&Wire);
Adafruit_LIS3MDL lis3;
Adafruit_LSM6DSOX sox;
extern TwoWire Wire1;
IMURaw imu;

SemaphoreHandle_t state_mutex;

inline int16_t gyro_dps_q15(int16_t raw) {
    return (raw * GYRO_SCALE_Q15) >> 15;
}

void setrgb(uint8_t red, uint8_t green, uint8_t blue)
{
  neopixelWrite(RGB_PIN, red, green, blue);
}

uint32_t servoMicrosecondsToDuty(uint16_t microseconds)
{
    const uint32_t max_duty = (1 << PWM_RES_BITS) - 1;
    return (microseconds * max_duty) / 20000;
}

void setServoMicroseconds(uint8_t channel, uint16_t microseconds)
{
    if (microseconds < 1000) microseconds = 1000;
    if (microseconds > 2000) microseconds = 2000;
    uint32_t duty = servoMicrosecondsToDuty(microseconds);
    ledcWrite(channel, duty);
}

const uint32_t rate_period_us = 1000000 / RATE_LOOP_HZ;
const TickType_t period = pdMS_TO_TICKS(1000 / RATE_LOOP_HZ);
TickType_t prev_wake = xTaskGetTickCount();

float lgx, lgy, lgz;
float lax, lay, laz;
float lmx, lmy, lmz;

uint32_t loops_in_window = 0;
uint32_t stats_window_start = 0;

sensors_event_t accel, gyro, temp, mag;

float current_pressure = 0.0f;
float current_temperature = 0.0f;
double seaLevelPressure = 0.0;

void setup() 
{
    Serial.begin(BAUD_RATE);
    delay(3000); 
    Serial.println("\n\nInitializing peripherals...\n\n");
    setrgb(255, 255, 0);
    
    Wire.setPins(SDA_IMU_PIN, SCL_IMU_PIN);
    Wire.begin();
    Wire1.setPins(SDA_MAG_PIN, SCL_MAG_PIN);
    Wire1.begin();
    delay(3000);
    Wire.setClock(400000);

    // ===== LSM6DSOX INIT =====
    Serial.print("Initializing LSM6DSOX sensor...");
    if (!sox.begin_I2C(0x6B))
    {
        Serial.print("FAILED.\n");
        setrgb(255, 0, 0);
        while (true) delay(1);
    }
    Serial.print("OK.\n");
    
    // ===== CONFIGURE LSM6DSOX (CRITICAL!) =====
    sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    sox.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
    sox.setAccelDataRate(LSM6DS_RATE_833_HZ);
    sox.setGyroDataRate(LSM6DS_RATE_833_HZ);
    
    Serial.printf("  Accel: ±4G @ 104Hz\n");
    Serial.printf("  Gyro: ±500dps @ 104Hz\n");
    
    delay(3000);

    // ===== LIS3MDL INIT =====
    Serial.print("Initializing LIS3MDL sensor...");
    if (!lis3.begin_I2C(0x1C, &Wire1))
    {
        Serial.print("FAILED.\n");
        setrgb(255, 0, 0);
        while (true) delay(1);
    }
    Serial.print("OK.\n");
    
    // ===== CONFIGURE LIS3MDL =====
    lis3.setRange(LIS3MDL_RANGE_4_GAUSS);
    lis3.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    
    Serial.printf("  Mag: ±4 Gauss @ 155Hz\n");

    setrgb(0, 255, 0);

    // ===== PERFORM CALIBRATIONS =====
    Serial.println("\n=== SENSOR CALIBRATION ===");
    
    // Gyro calibration (REQUIRED - FC must be still)
    if (!calibrator.calibrateGyro(sox, 500)) {
        Serial.println("Gyro calibration FAILED!");
        setrgb(255, 0, 0);
        while(1) delay(1);
    }
    
    // Magnetometer calibration
    Serial.println("\nChoose mag calibration method:");
    Serial.println("  [Press any key for ADVANCED calibration]");
    Serial.println("  [Or wait 5 sec for SIMPLE calibration]");
    
    unsigned long wait_start = millis();
    bool do_advanced = false;
    while (millis() - wait_start < 5000) {
        if (Serial.available()) {
            Serial.read();
            do_advanced = true;
            break;
        }
    }
    
    if (do_advanced) {
        if (!calibrator.calibrateMagAdvanced(lis3, 30)) {
            Serial.println("WARNING: Mag calibration failed!");
        }
    } else {
        if (!calibrator.calibrateMagSimple(lis3, 500)) {
            Serial.println("WARNING: Mag calibration failed!");
        }
    }
    
    Serial.println("\n=== CALIBRATION COMPLETE ===\n");

    setrgb(0, 255, 0);
    
    Serial.print("Initializing timer...");
    motor_init();
    Serial.print("OK.\n");

    Serial.print("Initializing UART CRSF receiver...");
    crsf_init();
    Serial.print("OK.\n");

    // ===== INITIALIZE MADGWICK FILTER =====
    // OPTION 1 (Adafruit):
    mad_filter.begin(450);

    mad_filter.setBeta(0.25f);
    
    // OPTION 2 (Custom): already initialized in declaration
    
    Serial.println("\n=== Flight Controller Ready ===\n");
}

void loop() 
{
    bool ready = true;
    uint32_t loop_start = micros();

    // === GRAB SENSOR DATA ===
    uint32_t t0 = micros();
    
    // if (!sox.getEvent(&accel, &gyro, &temp))
    // {
    //     Serial.println("\n*** Failed to get SOX event ***\n");
    //     ready = false;
    // }
    // if (!lis3.getEvent(&mag)) 
    // {
    //     Serial.println("\n*** Failed to get LIS3 event ***\n");
    //     ready = false;
    // }

    lsm6_ll_read(imu);

    // uint32_t imu_read_us = micros() - t0;

    // Serial.printf("IMU read time: %lu us\n", imu_read_us);

    if (ready)
    {
        //uint32_t t0 = micros();

        // Get RAW sensor values
        float gx_raw = imu.gx * GYRO_SCALE;
        float gy_raw = imu.gy * GYRO_SCALE;
        float gz_raw = imu.gz * GYRO_SCALE;

        float ax_raw = imu.ax * ACC_SCALE;
        float ay_raw = imu.ay * ACC_SCALE;
        float az_raw = imu.az * ACC_SCALE;
        
        // float mx_raw = mag.magnetic.x;
        // float my_raw = mag.magnetic.y;
        // float mz_raw = mag.magnetic.z;

        calibrator.applyGyroCalibration(gx_raw, gy_raw, gz_raw);
        //calibrator.applyMagCalibration(mx_raw, my_raw, mz_raw);
        
        // ===== APPLY AXIS REMAPPING =====
        SensorAxes::remapGyro(gx_raw, gy_raw, gz_raw, lgx, lgy, lgz);
        SensorAxes::remapAccel(ax_raw, ay_raw, az_raw, lax, lay, laz);
        //SensorAxes::remapMag(mx_raw, my_raw, mz_raw, lmx, lmy, lmz);

        // ===== UPDATE FILTER =====
        mad_filter.updateIMU(gx_raw, gy_raw, gz_raw, ax_raw, ay_raw, az_raw);
        // OR for IMU-only: mad_filter.updateIMU(lgx, lgy, lgz, lax, lay, laz);

        float roll = mad_filter.getRoll();
        float pitch = mad_filter.getPitch();
        float yaw = mad_filter.getYaw();  

        uint32_t imu_time_us = micros() - t0;

        // Serial.printf("IMU time: %lu us\n", imu_time_us);

        // Serial.printf("\nRoll: %.2f, Pitch: %.2f\n", roll, pitch);

        // Serial.print(roll);
        // Serial.print(',');
        // Serial.print(pitch);
        // Serial.print(',');
        // Serial.println(yaw);

        static uint32_t last_print = 0;
        if (millis() - last_print > 100) {
            last_print = millis();
            // Serial.printf("Pitch: %.2f Roll: %.2f Yaw: %.2f\n", pitch, roll, yaw);
        }

        state.roll = roll;
        state.pitch = pitch;
        state.yaw = yaw;
        state.gyro_x = lgx;
        state.gyro_y = lgy;
        state.gyro_z = lgz;
        state.timestamp_ms = millis();
        state.data_ready = true;
    }

    // uint32_t t1 = micros();

    // // === CONTROL ===
    // crsf_update();
    
    // uint32_t t2 = micros() - t1;

    // if (motor_arm())
    // {
    //     int16_t rx_throttle = crsf_get_channel(2);
    //     int16_t rx_roll = crsf_get_channel(0);
    //     int16_t rx_pitch = crsf_get_channel(1);
    //     int16_t rx_yaw = crsf_get_channel(3);

    //     auto norm = [](int16_t val)
    //     {
    //         return (float)(val - 992) / 820.0f;
    //     };
    //     float roll = norm(rx_roll);
    //     float pitch = norm(rx_pitch);
    //     float yaw = norm(rx_yaw);
    //     float throttle = (float)(rx_throttle - 172) / (1811 - 172);

    //     static int print_counter = 0;
    //     if (++print_counter >= 10)
    //     {
    //         print_counter = 0;
    //         // Serial.printf("[RX] Thr:%d  Roll:%d  Pitch:%d  Yaw:%d\n", 
    //         //               rx_throttle, rx_roll, rx_pitch, rx_yaw);
    //     }

    //     motor_update_from_crsf();
    // }

    // Serial.printf("Control update time: %lu us\n", t2);

    // === METRICS ===
    //perf.mpu_read_us = imu_read_us;
    perf.loop_time_us = micros() - loop_start;
    loops_in_window++;

    // Serial.printf("loop_time_us = %lu\n", perf.loop_time_us);
}