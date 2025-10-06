#ifndef EKF_H
#define EKF_H

#include <Arduino.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

// =============================
// EKF STRUCTURE
// =============================

struct EKF {
    // --- state vector ---
    Matrix<7, 1, float> x;      // state vector [q0,q1,q2,q3,bx,by,bz]
    Matrix<7, 7, float> P;      // state covariance
    Matrix<7, 7, float> Q;      // process noise
    Matrix<6, 6, float> R;      // measurement noise
    // TODO: we might have to tune initial values of Q and R empirically later on... 

    float dt; // define the time step in seconds

    EKF(float dt = 0.01f);

    void predict(const BLA::Matrix<3>& gyro);
    void update(const BLA::Matrix<3>& accel, const BLA::Matrix<3>& mag);

    void normalizeQuaternion();
    void printState();
};

#endif // EKF_H