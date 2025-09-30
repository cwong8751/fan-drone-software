#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include <BasicLinearAlgebra.h>
#include <cmath>

#define RGB_pin 48

void setrgb(uint8_t red, uint8_t green, uint8_t blue){
  neopixelWrite(RGB_pin, red, green, blue);
}

class OrientationEKF {
private:
    // State vector: [q0, q1, q2, q3, bias_x, bias_y, bias_z]
    // q0-q3 are quaternion components, bias_x/y/z are gyro biases
    BLA::Matrix<7, 1> x;  // State vector (reduced from 10 to 7)
    BLA::Matrix<7, 7> P;  // Covariance matrix
    BLA::Matrix<7, 7> Q;  // Process noise covariance
    BLA::Matrix<3, 3> R_acc;  // Accelerometer measurement noise
    BLA::Matrix<3, 3> R_mag;  // Magnetometer measurement noise
    
    float dt;  // Time step
    
    // Magnetic declination for your location (radians)
    float mag_declination = 14.0 * PI / 180.0f;
    
    // Low-pass filter for linear acceleration detection
    float acc_magnitude_filtered = 9.81f;
    const float acc_filter_alpha = 0.1f;
    
public:
    OrientationEKF() {
        // Initialize state vector (identity quaternion, zero biases)
        x(0) = 1.0f;  // q0 (w component)
        x(1) = 0.0f;  // q1 (x component)
        x(2) = 0.0f;  // q2 (y component)
        x(3) = 0.0f;  // q3 (z component)
        x(4) = 0.0f;  // bias_x
        x(5) = 0.0f;  // bias_y
        x(6) = 0.0f;  // bias_z
        
        // Initialize covariance matrix
        P.Fill(0);
        for (int i = 0; i < 4; i++) P(i, i) = 0.1f;  // Quaternion uncertainty
        for (int i = 4; i < 7; i++) P(i, i) = 0.01f; // Bias uncertainty

        // Process noise covariance - tuned values
        Q.Fill(0);
        Q(0, 0) = 0.0001f;  // q0 process noise
        Q(1, 1) = 0.0001f;  // q1 process noise
        Q(2, 2) = 0.0001f;  // q2 process noise
        Q(3, 3) = 0.0001f;  // q3 process noise
        Q(4, 4) = 1e-8f;    // bias_x process noise (very small)
        Q(5, 5) = 1e-8f;    // bias_y process noise
        Q(6, 6) = 1e-8f;    // bias_z process noise

        // Measurement noise covariance - tuned values
        R_acc.Fill(0);
        R_acc(0, 0) = R_acc(1, 1) = R_acc(2, 2) = 0.3f;  // Accelerometer noise
        
        R_mag.Fill(0);
        R_mag(0, 0) = R_mag(1, 1) = R_mag(2, 2) = 0.1f;  // Magnetometer noise
        
        dt = 0.01f;  // Default 100Hz
    }
    
    void setDeltaTime(float deltaTime) {
        dt = deltaTime;
    }
    
    void setMagneticDeclination(float declination_deg) {
        mag_declination = declination_deg * PI / 180.0f;
    }
    
    // Normalize quaternion
    void normalizeQuaternion() {
        float norm = sqrt(x(0)*x(0) + x(1)*x(1) + x(2)*x(2) + x(3)*x(3));
        if (norm > 0.0001f) {
            x(0) /= norm;
            x(1) /= norm;
            x(2) /= norm;
            x(3) /= norm;
        } else {
            // Reset to identity quaternion if norm is too small
            x(0) = 1.0f;
            x(1) = x(2) = x(3) = 0.0f;
        }
    }
    
    // Prediction step
    void predict(float gx, float gy, float gz) {
        // Remove bias from gyro measurements
        float wx = gx - x(4);
        float wy = gy - x(5);
        float wz = gz - x(6);
        
        // Current quaternion
        float q0 = x(0), q1 = x(1), q2 = x(2), q3 = x(3);
        
        // Quaternion derivative (corrected signs)
        float dq0 = -0.5f * (q1*wx + q2*wy + q3*wz);
        float dq1 =  0.5f * (q0*wx + q2*wz - q3*wy);
        float dq2 =  0.5f * (q0*wy - q1*wz + q3*wx);
        float dq3 =  0.5f * (q0*wz + q1*wy - q2*wx);
        
        // Update quaternion state (Euler integration)
        x(0) += dq0 * dt;
        x(1) += dq1 * dt;
        x(2) += dq2 * dt;
        x(3) += dq3 * dt;
        // Biases remain constant in prediction
        
        normalizeQuaternion();
        
        // State transition Jacobian F
        BLA::Matrix<7, 7> F_mat;
        F_mat.Fill(0);
        
        // Identity for bias states
        F_mat(4, 4) = 1.0f; F_mat(5, 5) = 1.0f; F_mat(6, 6) = 1.0f;
        
        // Quaternion state transition jacobian
        F_mat(0, 0) = 1.0f; F_mat(0, 1) = -0.5f*wx*dt; F_mat(0, 2) = -0.5f*wy*dt; F_mat(0, 3) = -0.5f*wz*dt;
        F_mat(1, 0) = 0.5f*wx*dt; F_mat(1, 1) = 1.0f; F_mat(1, 2) = 0.5f*wz*dt; F_mat(1, 3) = -0.5f*wy*dt;
        F_mat(2, 0) = 0.5f*wy*dt; F_mat(2, 1) = -0.5f*wz*dt; F_mat(2, 2) = 1.0f; F_mat(2, 3) = 0.5f*wx*dt;
        F_mat(3, 0) = 0.5f*wz*dt; F_mat(3, 1) = 0.5f*wy*dt; F_mat(3, 2) = -0.5f*wx*dt; F_mat(3, 3) = 1.0f;
        
        // Partial derivatives w.r.t. biases
        F_mat(0, 4) = 0.5f*q1*dt; F_mat(0, 5) = 0.5f*q2*dt; F_mat(0, 6) = 0.5f*q3*dt;
        F_mat(1, 4) = -0.5f*q0*dt; F_mat(1, 5) = 0.5f*q3*dt; F_mat(1, 6) = -0.5f*q2*dt;
        F_mat(2, 4) = -0.5f*q3*dt; F_mat(2, 5) = -0.5f*q0*dt; F_mat(2, 6) = 0.5f*q1*dt;
        F_mat(3, 4) = 0.5f*q2*dt; F_mat(3, 5) = -0.5f*q1*dt; F_mat(3, 6) = -0.5f*q0*dt;
        
        // Update covariance
        P = F_mat * P * (~F_mat) + Q;
    }
    
    // Convert quaternion to rotation matrix (body to world)
    BLA::Matrix<3, 3> quaternionToRotationMatrix() {
        BLA::Matrix<3, 3> R;
        float q0 = x(0), q1 = x(1), q2 = x(2), q3 = x(3);
        
        // Body to world rotation matrix
        R(0, 0) = 1 - 2*(q2*q2 + q3*q3);
        R(0, 1) = 2*(q1*q2 - q0*q3);
        R(0, 2) = 2*(q1*q3 + q0*q2);
        R(1, 0) = 2*(q1*q2 + q0*q3);
        R(1, 1) = 1 - 2*(q1*q1 + q3*q3);
        R(1, 2) = 2*(q2*q3 - q0*q1);
        R(2, 0) = 2*(q1*q3 - q0*q2);
        R(2, 1) = 2*(q2*q3 + q0*q1);
        R(2, 2) = 1 - 2*(q1*q1 + q2*q2);
        
        return R;
    }
    
    // Update with accelerometer data
    void updateAccelerometer(float ax, float ay, float az) {
        // Check if we're experiencing significant linear acceleration
        float acc_magnitude = sqrt(ax*ax + ay*ay + az*az);
        acc_magnitude_filtered = acc_filter_alpha * acc_magnitude + (1.0f - acc_filter_alpha) * acc_magnitude_filtered;
        
        // Skip accelerometer update if experiencing high linear acceleration
        if (abs(acc_magnitude - 1.0f) > 0.3f) {  // 0.3g threshold
            return;
        }
        
        // Normalize accelerometer measurement
        if (acc_magnitude > 0.1f) {
            ax /= acc_magnitude;
            ay /= acc_magnitude;
            az /= acc_magnitude;
        }
        
        // Expected gravity vector in body frame (world gravity rotated to body frame)
        BLA::Matrix<3, 3> R = quaternionToRotationMatrix();
        BLA::Matrix<3, 1> gravity_world = {0, 0, -1};  // Gravity points down in NED frame
        BLA::Matrix<3, 1> expected_acc = (~R) * gravity_world;  // World to body rotation
        
        // Measurement residual
        BLA::Matrix<3, 1> z = {ax, ay, az};
        BLA::Matrix<3, 1> innovation = z - expected_acc;
        
        // Measurement Jacobian H (3x7)
        BLA::Matrix<3, 7> H;
        H.Fill(0);
        
        float q0 = x(0), q1 = x(1), q2 = x(2), q3 = x(3);
        
        // Partial derivatives of expected gravity w.r.t. quaternion
        // These are the derivatives of R^T * [0;0;-1] w.r.t. quaternion
        H(0, 0) = -2*q2; H(0, 1) = -2*q3; H(0, 2) = -2*q0; H(0, 3) = -2*q1;
        H(1, 0) = 2*q1;  H(1, 1) = 2*q0;  H(1, 2) = -2*q3; H(1, 3) = -2*q2;
        H(2, 0) = -2*q0; H(2, 1) = 2*q1;  H(2, 2) = 2*q2;  H(2, 3) = -2*q3;
        
        // Innovation covariance
        BLA::Matrix<3, 3> S = H * P * (~H) + R_acc;
        
        // Kalman gain
        BLA::Matrix<7, 3> K = P * (~H) * Inverse(S);
        
        // Update state and covariance
        x = x + K * innovation;
        BLA::Matrix<7, 7> I; 
        I.Fill(0); 
        for(int i = 0; i < 7; i++) I(i, i) = 1.0f;
        P = (I - K * H) * P;
        
        normalizeQuaternion();
    }
    
    // Update with magnetometer data
    void updateMagnetometer(float mx, float my, float mz) {
        // Normalize magnetometer measurement
        float norm = sqrt(mx*mx + my*my + mz*mz);
        if (norm < 0.1f) return;  // Skip if measurement is too weak
        mx /= norm; my /= norm; mz /= norm;
        
        // Expected magnetic field in body frame
        BLA::Matrix<3, 3> R = quaternionToRotationMatrix();
        
        // Magnetic field in world frame (North-East-Down convention)
        // Adjusted for magnetic declination
        float cos_dec = cos(mag_declination);
        float sin_dec = sin(mag_declination);
        BLA::Matrix<3, 1> mag_world = {cos_dec, sin_dec, 0};  // Horizontal magnetic field
        BLA::Matrix<3, 1> expected_mag = (~R) * mag_world;  // Rotate to body frame
        
        // Measurement residual
        BLA::Matrix<3, 1> z = {mx, my, mz};
        BLA::Matrix<3, 1> innovation = z - expected_mag;
        
        // Measurement Jacobian H (3x7)
        BLA::Matrix<3, 7> H;
        H.Fill(0);
        
        float q0 = x(0), q1 = x(1), q2 = x(2), q3 = x(3);
        
        // Partial derivatives of expected magnetic field w.r.t. quaternion
        H(0, 0) = -2*(q2*cos_dec + q3*sin_dec);
        H(0, 1) = -2*(q3*cos_dec - q2*sin_dec);
        H(0, 2) = -2*(q0*cos_dec + q1*sin_dec);
        H(0, 3) = -2*(q1*cos_dec - q0*sin_dec);
        
        H(1, 0) = 2*(q1*cos_dec + q0*sin_dec);
        H(1, 1) = 2*(q0*cos_dec - q1*sin_dec);
        H(1, 2) = 2*(q3*cos_dec + q2*sin_dec);
        H(1, 3) = 2*(q2*cos_dec - q3*sin_dec);
        
        H(2, 0) = -2*(q0*cos_dec - q1*sin_dec);
        H(2, 1) = 2*(q1*cos_dec + q0*sin_dec);
        H(2, 2) = -2*(q2*cos_dec - q3*sin_dec);
        H(2, 3) = 2*(q3*cos_dec + q2*sin_dec);
        
        // Innovation covariance
        BLA::Matrix<3, 3> S = H * P * (~H) + R_mag;
        
        // Kalman gain
        BLA::Matrix<7, 3> K = P * (~H) * Inverse(S);
        
        // Update state and covariance
        x = x + K * innovation;
        BLA::Matrix<7, 7> I; 
        I.Fill(0); 
        for(int i = 0; i < 7; i++) I(i, i) = 1.0f;
        P = (I - K * H) * P;
        
        normalizeQuaternion();
    }
    
    // Get current orientation as Euler angles (roll, pitch, yaw in radians)
    void getEulerAngles(float &roll, float &pitch, float &yaw) {
        float q0 = x(0), q1 = x(1), q2 = x(2), q3 = x(3);
        
        // Standard aerospace sequence (ZYX)
        roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
        
        float sin_pitch = 2*(q0*q2 - q3*q1);
        sin_pitch = constrain(sin_pitch, -1.0f, 1.0f);  // Clamp to avoid NaN
        pitch = asin(sin_pitch);
        
        yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
    }
    
    // Get current quaternion
    void getQuaternion(float &q0, float &q1, float &q2, float &q3) {
        q0 = x(0); q1 = x(1); q2 = x(2); q3 = x(3);
    }
    
    // Get gyro biases
    void getGyroBiases(float &bias_x, float &bias_y, float &bias_z) {
        bias_x = x(4); bias_y = x(5); bias_z = x(6);
    }
};

// Global variables
MPU9250 mpu;
OrientationEKF ekf;
unsigned long lastTime = 0;

void setup() {
    setrgb(255, 0, 0);
    Serial.begin(115200);
    Wire.begin(8, 9);
    delay(2000);
    
    // Initialize MPU9250
    if (!mpu.setup(0x68)) {
        Serial.println("MPU9250 connection failed");
        setrgb(255, 0, 0);  // Red for error
        while (1) delay(100);
    }
    
    // Calibrate gyro and accel
    Serial.println("Calibrating sensors... Keep the device still!");
    setrgb(255, 255, 0);  // Yellow for calibration
    delay(1000);
    mpu.calibrateAccelGyro();
    
    Serial.println("Calibrating magnetometer... Rotate the device in figure-8 pattern!");
    setrgb(0, 0, 255);  // Blue for mag calibration
    delay(1000);
    mpu.calibrateMag();
    
    Serial.println("Calibration complete!");
    
    // Set magnetic declination for your location
    // Find yours at: https://www.magnetic-declination.com/
    ekf.setMagneticDeclination(14.0);  // Update this value for your location!
    
    setrgb(0, 255, 0);  // Green for ready
    lastTime = micros();
}

void loop() {
    static unsigned long nextUpdateTime = 0;
    const unsigned long UPDATE_INTERVAL_US = 10000;  // 10ms = 100Hz
    
    unsigned long currentTime = micros();
    
    // Only run if it's time for the next update
    if (currentTime >= nextUpdateTime) {
        // Calculate actual dt
        float dt = (currentTime - lastTime) / 1000000.0f;
        lastTime = currentTime;
        
        // Schedule next update
        nextUpdateTime = currentTime + UPDATE_INTERVAL_US;
        
        // Update sensor data
        if (mpu.update()) {
            // Get sensor readings (convert gyro to rad/s)
            float gx = mpu.getGyroX() * PI / 180.0f;
            float gy = mpu.getGyroY() * PI / 180.0f;
            float gz = mpu.getGyroZ() * PI / 180.0f;
            
            // Get accelerometer readings (already in g's)
            float ax = mpu.getAccX();
            float ay = mpu.getAccY();
            float az = mpu.getAccZ();
            
            // Get magnetometer readings
            float mx = mpu.getMagX();
            float my = mpu.getMagY();
            float mz = mpu.getMagZ();
            
            // Set time step and run EKF
            ekf.setDeltaTime(dt);
            
            // Prediction step with gyro data
            ekf.predict(gx, gy, gz);
            
            // Update with accelerometer
            ekf.updateAccelerometer(ax, ay, az);
            
            // Update with magnetometer at reduced rate
            static int mag_counter = 0;
            if (++mag_counter >= 5) {  // Update magnetometer every 5th iteration (20Hz)
                ekf.updateMagnetometer(mx, my, mz);
                mag_counter = 0;
            }
            
            // Get orientation and convert to degrees
            float roll, pitch, yaw;
            ekf.getEulerAngles(roll, pitch, yaw);
            
            roll *= 180.0f / PI;
            pitch *= 180.0f / PI;
            yaw *= 180.0f / PI;



            roll = atan2(ay, az) * 180.0f / PI;
            pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0f / PI;
            



            
            // Print results in the expected format
            Serial.print(roll, 2); Serial.print("\t");
            Serial.print(pitch, 2); Serial.print("\t");
            Serial.println(yaw, 2);
        }
    }
    
    // Yield to other tasks
    yield();
}