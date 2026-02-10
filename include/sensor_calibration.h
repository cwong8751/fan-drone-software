#ifndef SENSOR_CALIBRATION_H
#define SENSOR_CALIBRATION_H

#include <Arduino.h>
#include "Adafruit_LSM6DSOX.h"
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

// Calibration data storage
struct GyroCalibration {
    float offset_x = 0.0f;
    float offset_y = 0.0f;
    float offset_z = 0.0f;
    bool calibrated = false;
};

struct MagCalibration {
    // Hard iron offsets (bias)
    float offset_x = 0.0f;
    float offset_y = 0.0f;
    float offset_z = 0.0f;
    
    // Soft iron correction matrix (scale factors)
    float scale_x = 1.0f;
    float scale_y = 1.0f;
    float scale_z = 1.0f;
    
    bool calibrated = false;
};

struct AccelCalibration {
    float offset_x = 0.0f;
    float offset_y = 0.0f;
    float offset_z = 0.0f;
    bool calibrated = false;
};

class SensorCalibrator {
public:
    GyroCalibration gyro_cal;
    MagCalibration mag_cal;
    AccelCalibration accel_cal;
    
    // Calibrate gyroscope (FC must be stationary!)
    bool calibrateGyro(Adafruit_LSM6DSOX &imu, int samples = 500) {
        Serial.println("\n=== GYRO CALIBRATION ===");
        Serial.println("Place FC on FLAT, STABLE surface.");
        Serial.println("DO NOT MOVE for 5 seconds...");
        delay(5000);
        
        Serial.print("Collecting ");
        Serial.print(samples);
        Serial.println(" samples...");
        
        float sum_x = 0, sum_y = 0, sum_z = 0;
        sensors_event_t accel, gyro, temp;
        
        for (int i = 0; i < samples; i++) {
            if (!imu.getEvent(&accel, &gyro, &temp)) {
                Serial.println("ERROR: Failed to read gyro during calibration!");
                return false;
            }
            
            sum_x += gyro.gyro.x;
            sum_y += gyro.gyro.y;
            sum_z += gyro.gyro.z;
            
            // Progress indicator
            if (i % 50 == 0) {
                Serial.print(".");
            }
            
            delay(10); // 10ms between samples
        }
        
        gyro_cal.offset_x = sum_x / samples;
        gyro_cal.offset_y = sum_y / samples;
        gyro_cal.offset_z = sum_z / samples;
        gyro_cal.calibrated = true;
        
        Serial.println("\n✓ Gyro calibration complete!");
        Serial.printf("  Offsets: X=%.4f, Y=%.4f, Z=%.4f rad/s\n", 
                      gyro_cal.offset_x, gyro_cal.offset_y, gyro_cal.offset_z);
        
        return true;
    }
    
    // Simple magnetometer calibration (hard iron only)
    // For better results, use full 3D calibration with rotation
    bool calibrateMagSimple(Adafruit_LIS3MDL &mag, int samples = 500) {
        Serial.println("\n=== MAG CALIBRATION (Simple) ===");
        Serial.println("Collecting baseline samples...");
        Serial.println("Keep FC stationary for 5 seconds...");
        delay(5000);
        
        float sum_x = 0, sum_y = 0, sum_z = 0;
        sensors_event_t mag_event;
        
        for (int i = 0; i < samples; i++) {
            if (!mag.getEvent(&mag_event)) {
                Serial.println("ERROR: Failed to read mag during calibration!");
                return false;
            }
            
            sum_x += mag_event.magnetic.x;
            sum_y += mag_event.magnetic.y;
            sum_z += mag_event.magnetic.z;
            
            if (i % 50 == 0) Serial.print(".");
            delay(10);
        }
        
        mag_cal.offset_x = sum_x / samples;
        mag_cal.offset_y = sum_y / samples;
        mag_cal.offset_z = sum_z / samples;
        mag_cal.calibrated = true;
        
        Serial.println("\n✓ Mag calibration complete (simple mode)!");
        Serial.printf("  Offsets: X=%.2f, Y=%.2f, Z=%.2f uT\n", 
                      mag_cal.offset_x, mag_cal.offset_y, mag_cal.offset_z);
        Serial.println("  NOTE: For best results, perform full 3D calibration!");
        
        return true;
    }
    
    // Advanced magnetometer calibration - rotate FC in all axes
    bool calibrateMagAdvanced(Adafruit_LIS3MDL &mag, int duration_sec = 30) {
        Serial.println("\n=== MAG CALIBRATION (Advanced) ===");
        Serial.println("Slowly rotate FC in ALL directions for 30 seconds:");
        Serial.println("  - Tumble it");
        Serial.println("  - Flip it");
        Serial.println("  - Spin it");
        Serial.println("Starting in 3 seconds...");
        delay(3000);
        
        float min_x = 1e6, max_x = -1e6;
        float min_y = 1e6, max_y = -1e6;
        float min_z = 1e6, max_z = -1e6;
        
        sensors_event_t mag_event;
        unsigned long start = millis();
        int sample_count = 0;
        
        while (millis() - start < (duration_sec * 1000)) {
            if (!mag.getEvent(&mag_event)) continue;
            
            float mx = mag_event.magnetic.x;
            float my = mag_event.magnetic.y;
            float mz = mag_event.magnetic.z;
            
            if (mx < min_x) min_x = mx;
            if (mx > max_x) max_x = mx;
            if (my < min_y) min_y = my;
            if (my > max_y) max_y = my;
            if (mz < min_z) min_z = mz;
            if (mz > max_z) max_z = mz;
            
            sample_count++;
            
            // Progress indicator every second
            static unsigned long last_print = 0;
            if (millis() - last_print > 1000) {
                last_print = millis();
                int elapsed = (millis() - start) / 1000;
                Serial.printf("  %d/%d sec (samples: %d)\n", 
                              elapsed, duration_sec, sample_count);
            }
            
            delay(10);
        }
        
        // Hard iron correction (offset to center)
        mag_cal.offset_x = (max_x + min_x) / 2.0f;
        mag_cal.offset_y = (max_y + min_y) / 2.0f;
        mag_cal.offset_z = (max_z + min_z) / 2.0f;
        
        // Soft iron correction (scale to sphere)
        float range_x = max_x - min_x;
        float range_y = max_y - min_y;
        float range_z = max_z - min_z;
        float avg_range = (range_x + range_y + range_z) / 3.0f;
        
        mag_cal.scale_x = avg_range / range_x;
        mag_cal.scale_y = avg_range / range_y;
        mag_cal.scale_z = avg_range / range_z;
        
        mag_cal.calibrated = true;
        
        Serial.println("\n✓ Advanced mag calibration complete!");
        Serial.printf("  Hard Iron Offsets: X=%.2f, Y=%.2f, Z=%.2f uT\n", 
                      mag_cal.offset_x, mag_cal.offset_y, mag_cal.offset_z);
        Serial.printf("  Soft Iron Scales: X=%.3f, Y=%.3f, Z=%.3f\n", 
                      mag_cal.scale_x, mag_cal.scale_y, mag_cal.scale_z);
        Serial.printf("  Ranges: X=%.1f, Y=%.1f, Z=%.1f uT\n", 
                      range_x, range_y, range_z);
        
        return true;
    }
    
    // Apply calibrations to raw sensor data
    void applyGyroCalibration(float &gx, float &gy, float &gz) {
        if (gyro_cal.calibrated) {
            gx -= gyro_cal.offset_x;
            gy -= gyro_cal.offset_y;
            gz -= gyro_cal.offset_z;
        }
    }
    
    void applyMagCalibration(float &mx, float &my, float &mz) {
        if (mag_cal.calibrated) {
            // Hard iron correction
            mx -= mag_cal.offset_x;
            my -= mag_cal.offset_y;
            mz -= mag_cal.offset_z;
            
            // Soft iron correction
            mx *= mag_cal.scale_x;
            my *= mag_cal.scale_y;
            mz *= mag_cal.scale_z;
        }
    }
    
    void applyAccelCalibration(float &ax, float &ay, float &az) {
        if (accel_cal.calibrated) {
            ax -= accel_cal.offset_x;
            ay -= accel_cal.offset_y;
            az -= accel_cal.offset_z;
        }
    }
};

#endif