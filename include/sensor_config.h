#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

#include <Arduino.h>

// Sensor axis remapping configuration
// Adjust these based on your physical sensor mounting orientation
struct SensorAxes {
    // Gyroscope remapping (body frame)
    static inline void remapGyro(float gx_raw, float gy_raw, float gz_raw,
                                  float &gx_body, float &gy_body, float &gz_body) {
        // Default: no remapping (sensor aligned with body frame)
        gx_body = gx_raw;
        gy_body = gy_raw;
        gz_body = gz_raw;
        
        // Example from reference code (uncomment if needed):
        // gx_body = gz_raw;   // Roll_body from Yaw_imu
        // gy_body = gy_raw;   // Pitch_body from Pitch_imu
        // gz_body = gx_raw;   // Yaw_body from Roll_imu
    }
    
    // Accelerometer remapping (body frame)
    static inline void remapAccel(float ax_raw, float ay_raw, float az_raw,
                                   float &ax_body, float &ay_body, float &az_body) {
        // Default: no remapping
        ax_body = ax_raw;
        ay_body = ay_raw;
        az_body = az_raw;
        
        // Example from reference code (uncomment if needed):
        // ax_body = az_raw;   // X_body from Y_imu
        // ay_body = ax_raw;   // Y_body from Z_imu
        // az_body = ay_raw;   // Z_body from X_imu
    }
    
    // Magnetometer remapping (body frame)
    static inline void remapMag(float mx_raw, float my_raw, float mz_raw,
                                 float &mx_body, float &my_body, float &mz_body) {
        // Default: no remapping
        mx_body = mx_raw;
        my_body = my_raw;
        mz_body = mz_raw;
        
        // Example from reference code (uncomment if needed):
        // mx_body = mz_raw;    // X_body from Z_mag
        // my_body = -my_raw;   // Y_body from -Y_mag
        // mz_body = mx_raw;    // Z_body from X_mag
    }
};

#endif // SENSOR_CONFIG_H