#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include <Arduino.h>

class MadgwickFilter {
private:
    // Quaternion state (w, x, y, z)
    float q0, q1, q2, q3;
    
    // Filter parameters
    float beta;  // Algorithm gain
    unsigned long lastUpdateTime;
    
    // Euler angles (output)
    float roll, pitch, yaw;
    
public:
    MadgwickFilter(float sampleFreqHz = 100.0f, float betaGain = 0.1f) {
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
        
        beta = betaGain;
        
        lastUpdateTime = 0;
        roll = pitch = yaw = 0.0f;
    }
    
    // Update with gyro + accel + mag (9-DOF)
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz) {
        
        unsigned long now = micros();
        
        // FIRST CALL - INITIALIZE FROM ACCELEROMETER
        if (lastUpdateTime == 0) {
            lastUpdateTime = now;
            
            // Get initial orientation from gravity
            float norm = sqrtf(ax*ax + ay*ay + az*az);
            if (norm > 0) {
                float ax_norm = ax / norm;
                float ay_norm = ay / norm;
                float az_norm = az / norm;
                
                // Initialize angles from accelerometer
                roll = atan2f(ay_norm, az_norm) * 180.0f / M_PI;
                pitch = atan2f(-ax_norm, sqrtf(ay_norm*ay_norm + az_norm*az_norm)) * 180.0f / M_PI;
                yaw = 0.0f;
                
                Serial.printf("[FILTER INIT 9DOF] Roll: %.2f, Pitch: %.2f\n", roll, pitch);
            }
            return;
        }
        
        float dt = (now - lastUpdateTime) / 1e6f;
        lastUpdateTime = now;
        
        // Sanity check dt
        if (dt > 0.1f) {
            Serial.println("[FILTER] WARNING: dt capped at 0.1s");
            dt = 0.1f;
        }
        if (dt < 0.0001f) {  // 100µs minimum
            Serial.printf("[FILTER] dt too small (%.6f s), skipping!\n", dt);
            return;
        }
        
        // Normalize accelerometer
        float norm = sqrtf(ax*ax + ay*ay + az*az);
        if (norm == 0.0f) {
            Serial.println("[FILTER] WARNING: Zero accel vector!");
            return;
        }
        ax /= norm;
        ay /= norm;
        az /= norm;
        
        // Normalize magnetometer
        norm = sqrtf(mx*mx + my*my + mz*mz);
        if (norm == 0.0f) {
            Serial.println("[FILTER] WARNING: Zero mag vector!");
            return;
        }
        mx /= norm;
        my /= norm;
        mz /= norm;
        
        // Reference direction of Earth's magnetic field
        float hx = 2.0f * (q0*q0 + q1*q1 - 0.5f) * mx + 
                   2.0f * (q1*q2 - q0*q3) * my + 
                   2.0f * (q1*q3 + q0*q2) * mz;
        float hy = 2.0f * (q1*q2 + q0*q3) * mx + 
                   2.0f * (q2*q2 + q0*q0 - 0.5f) * my + 
                   2.0f * (q2*q3 - q0*q1) * mz;
        float hz = 2.0f * (q1*q3 - q0*q2) * mx + 
                   2.0f * (q2*q3 + q0*q1) * my + 
                   2.0f * (q0*q0 + q3*q3 - 0.5f) * mz;
        float bx = sqrtf(hx*hx + hy*hy);
        float bz = hz;
        
        // Gradient descent algorithm corrective step
        float F[6] = {
            2.0f*(q1*q3 - q0*q2) - ax,
            2.0f*(q0*q1 + q2*q3) - ay,
            2.0f*(0.5f - q1*q1 - q2*q2) - az,
            2.0f*bx*(0.5f - q2*q2 - q3*q3) + 2.0f*bz*(q1*q3 - q0*q2) - mx,
            2.0f*bx*(q1*q2 - q0*q3) + 2.0f*bz*(q0*q1 + q2*q3) - my,
            2.0f*bx*(q0*q2 + q1*q3) + 2.0f*bz*(0.5f - q1*q1 - q2*q2) - mz
        };
        
        float J[6][4] = {
            {-2.0f*q2, 2.0f*q3, -2.0f*q0, 2.0f*q1},
            {2.0f*q1, 2.0f*q0, 2.0f*q3, 2.0f*q2},
            {0, -4.0f*q1, -4.0f*q2, 0},
            {-2.0f*bz*q3, 2.0f*bz*q2, -4.0f*bx*q3-2.0f*bz*q1, -4.0f*bx*q2+2.0f*bz*q0},
            {-2.0f*bx*q3+2.0f*bz*q1, 2.0f*bx*q2+2.0f*bz*q0, 2.0f*bx*q1+2.0f*bz*q3, -2.0f*bx*q0+2.0f*bz*q2},
            {2.0f*bx*q2, 2.0f*bx*q3-4.0f*bz*q1, 2.0f*bx*q0-4.0f*bz*q2, 2.0f*bx*q1}
        };
        
        float step[4] = {0, 0, 0, 0};
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 4; j++) {
                step[j] += J[i][j] * F[i];
            }
        }
        
        norm = sqrtf(step[0]*step[0] + step[1]*step[1] + step[2]*step[2] + step[3]*step[3]);
        if (norm > 0) {
            step[0] /= norm;
            step[1] /= norm;
            step[2] /= norm;
            step[3] /= norm;
        }
        
        // Compute rate of change of quaternion
        float dq0 = -0.5f * (q1*gx + q2*gy + q3*gz) - beta * step[0];
        float dq1 = 0.5f * (q0*gx + q2*gz - q3*gy) - beta * step[1];
        float dq2 = 0.5f * (q0*gy - q1*gz + q3*gx) - beta * step[2];
        float dq3 = 0.5f * (q0*gz + q1*gy - q2*gx) - beta * step[3];
        
        // Integrate quaternion
        q0 += dq0 * dt;
        q1 += dq1 * dt;
        q2 += dq2 * dt;
        q3 += dq3 * dt;
        
        // Normalize quaternion
        norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        if (norm > 0) {
            q0 /= norm;
            q1 /= norm;
            q2 /= norm;
            q3 /= norm;
        }
        
        // Convert to Euler angles
        roll = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * 180.0f / M_PI;
        pitch = asinf(2.0f*(q0*q2 - q3*q1)) * 180.0f / M_PI;
        yaw = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * 180.0f / M_PI;
    }
    
    // Update with gyro + accel only (6-DOF, no magnetometer)
    void updateIMU(float gx, float gy, float gz,
                   float ax, float ay, float az) {
        
        unsigned long now = micros();
        
        // FIRST CALL - INITIALIZE FROM ACCELEROMETER
        if (lastUpdateTime == 0) {
            lastUpdateTime = now;
            
            // Get initial orientation from gravity
            float norm = sqrtf(ax*ax + ay*ay + az*az);
            if (norm > 0) {
                float ax_norm = ax / norm;
                float ay_norm = ay / norm;
                float az_norm = az / norm;
                
                // Initialize angles from accelerometer
                roll = atan2f(ay_norm, az_norm) * 180.0f / M_PI;
                pitch = atan2f(-ax_norm, sqrtf(ay_norm*ay_norm + az_norm*az_norm)) * 180.0f / M_PI;
                yaw = 0.0f;
                
                Serial.printf("[FILTER INIT IMU] Roll: %.2f, Pitch: %.2f\n", roll, pitch);
            }
            return;
        }
        
        float dt = (now - lastUpdateTime) / 1e6f;
        lastUpdateTime = now;
        
        // Sanity check dt
        if (dt > 0.1f) {
            Serial.println("[FILTER] WARNING: dt capped at 0.1s");
            dt = 0.1f;
        }
        if (dt < 0.0001f) {  // 100µs minimum
            Serial.printf("[FILTER] dt too small (%.6f s), skipping!\n", dt);
            return;
        }
        
        // Normalize accelerometer
        float norm = sqrtf(ax*ax + ay*ay + az*az);
        if (norm == 0.0f) {
            Serial.println("[FILTER] WARNING: Zero accel vector!");
            return;
        }
        ax /= norm;
        ay /= norm;
        az /= norm;
        
        // Gradient descent for accel-only
        float F[3] = {
            2.0f*(q1*q3 - q0*q2) - ax,
            2.0f*(q0*q1 + q2*q3) - ay,
            2.0f*(0.5f - q1*q1 - q2*q2) - az
        };
        
        float J[3][4] = {
            {-2.0f*q2, 2.0f*q3, -2.0f*q0, 2.0f*q1},
            {2.0f*q1, 2.0f*q0, 2.0f*q3, 2.0f*q2},
            {0, -4.0f*q1, -4.0f*q2, 0}
        };
        
        float step[4] = {0, 0, 0, 0};
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                step[j] += J[i][j] * F[i];
            }
        }
        
        norm = sqrtf(step[0]*step[0] + step[1]*step[1] + step[2]*step[2] + step[3]*step[3]);
        if (norm > 0) {
            step[0] /= norm;
            step[1] /= norm;
            step[2] /= norm;
            step[3] /= norm;
        }
        
        // Compute rate of change of quaternion
        float dq0 = -0.5f * (q1*gx + q2*gy + q3*gz) - beta * step[0];
        float dq1 = 0.5f * (q0*gx + q2*gz - q3*gy) - beta * step[1];
        float dq2 = 0.5f * (q0*gy - q1*gz + q3*gx) - beta * step[2];
        float dq3 = 0.5f * (q0*gz + q1*gy - q2*gx) - beta * step[3];
        
        // Integrate quaternion
        q0 += dq0 * dt;
        q1 += dq1 * dt;
        q2 += dq2 * dt;
        q3 += dq3 * dt;
        
        // Normalize quaternion
        norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        if (norm > 0) {
            q0 /= norm;
            q1 /= norm;
            q2 /= norm;
            q3 /= norm;
        }
        
        // Convert to Euler angles
        roll = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * 180.0f / M_PI;
        pitch = asinf(2.0f*(q0*q2 - q3*q1)) * 180.0f / M_PI;
        yaw = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * 180.0f / M_PI;
    }
    
    // Getters
    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw() const { return yaw; }
    
    void getQuaternion(float &w, float &x, float &y, float &z) const {
        w = q0; x = q1; y = q2; z = q3;
    }
    
    // Setter for runtime tuning
    void setBeta(float b) { beta = b; }
    float getBeta() const { return beta; }
    
    // Get last dt for diagnostics
    float getLastDt() const { 
        if (lastUpdateTime == 0) return 0.0f;
        return (micros() - lastUpdateTime) / 1e6f; 
    }
};

#endif // MADGWICK_FILTER_H