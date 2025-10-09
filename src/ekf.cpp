#include "ekf.h"
#include <math.h>

using namespace BLA;

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0f)
#define RAD_TO_DEG(x) ((x) * 180.0f / M_PI)

// =============================
// CONSTRUCTOR (initial)
// =============================
EKF::EKF(float dt_) : dt(dt_) {
    x.Fill(0.0f);
    x(0) = 1.0f; // unit quaternion

    // initialize covariance and noise matrices
    P.Fill(0.0f);
    for (int i = 0; i < 7; i++) P(i,i) = 0.1f;

    Q.Fill(0.0f);
    for (int i = 0; i < 7; i++) Q(i,i) = 0.001f;

    R.Fill(0.0f);
    for (int i = 0; i < 6; i++) R(i,i) = 0.05f;
}

// =============================
// PREDICTION STEP (gyro input)
// 
// the EKF algorithm takes in our raw gyro measurements and quaternion state vector as input to make a prediction on the drone's orientation.
//
// in this function we predict the quaternion at step k of our EKF algorithm
//
// F matrix    -> state transition matrix
// P matrix    -> process noise matrix
// Q matrix    -> measurement noise matrix
// gyro matrix -> control vector
// x matrix    -> state vector
// =============================

void EKF::predict(const Matrix<3>& gyro) {
    float gx    =   gyro(0);
    float gy    =   gyro(1);
    float gz    =   gyro(2);

    // find the Jacobian of our nonlinear function using omega matrix at step k 
    // build our omega matrix (wacky way of initializing this matrix atm)
    BLA::Matrix<4,4> Omega;
    Omega(0,0) = 0.0f;  Omega(0,1) = -gx;   Omega(0,2) = -gy;   Omega(0,3) = -gz;
    Omega(1,0) = gx;    Omega(1,1) = 0.0f;  Omega(1,2) = gz;    Omega(1,3) = -gy;
    Omega(2,0) = gy;    Omega(2,1) = -gz;   Omega(2,2) = 0.0f;  Omega(2,3) = gx;
    Omega(3,0) = gz;    Omega(3,1) = gy;    Omega(3,2) = -gx;   Omega(3,3) = 0.0f;

    // grab our current quaternion (quaternion at step k-1)
    Matrix<4> q = { x(0), x(1), x(2), x(3) }; // current quaternion -> q_k-1 = [q_w, q_x, q_y, q_z]
    //q(0) = x(0); q(1) = x(1); q(2) = x(2); q(3) = x(3); <-- same thing twice?

    // find F to represent our Jacobian of our nonlinear function 
    // omega matrix (4x4) * q_k-1 (4x1) * 0.5 = q_dot (4x1)
    Matrix<4> q_dot = Omega * q;
    for (int i = 0; i < 4; i++) q_dot(i) += 0.5f;

    // normalize quaternion
    float norm = sqrtf(q(0)*q(0) + q(1)*q(1) + q(2)*q(2) + q(3)*q(3));
    if (norm > 1e-12f)
    {
        for (int i = 0; i < 4; i++) q(i) /= norm;
    }

    // update state
    x(0) = q(0); x(1) = q(1); x(2) = q(2); x(3) = q(3);

    // propagate covariance
    P = P + Q * dt; 
}

// =============================
// UPDATE STEP 
// =============================
void EKF::update(const BLA::Matrix<3>& accel, const BLA::Matrix<3>& mag) {
    // normalize sensor readings 
    BLA::Matrix<3> a = accel / sqrt(accel(0)*accel(0) + accel(1)*accel(1) + accel(2)*accel(2));
    BLA::Matrix<3> m = mag   / sqrt(mag(0)*mag(0)     + mag(1)*mag(1)     + mag(2)*mag(2));

    // get rotation matrix from quaternion
    float q0 = x(0), q1 = x(1), q2 = x(2), q3 = x(3);
    BLA::Matrix<3,3> Rb2w;
    Rb2w(0,0) = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    Rb2w(0,1) = 2*(q1*q2 - q0*q3);
    Rb2w(0,2) = 2*(q1*q3 + q0*q2);
    Rb2w(1,0) = 2*(q1*q2 + q0*q3);
    Rb2w(1,1) = q0*q0 - q1*q1 + q2*q2 - q3*q3;
    Rb2w(1,2) = 2*(q2*q3 - q0*q1);
    Rb2w(2,0) = 2*(q1*q3 - q0*q2);
    Rb2w(2,1) = 2*(q2*q3 + q0*q1);
    Rb2w(2,2) = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // predicted gravity and magnetic north in body frame
    BLA::Matrix<3> g_pred = Rb2w * BLA::Matrix<3>{0.0f, 0.0f, -1.0f};
    BLA::Matrix<3> m_pred = Rb2w * BLA::Matrix<3>{1.0f, 0.0f, 0.0f};


    // measurement residual 
    BLA::Matrix<6> z;
    for (int i = 0; i < 3; i++) z(i) = a(i) - g_pred(i);
    for (int i = 0; i < 3; i++) z(i+3) = m(i) - m_pred(i);

    P = P + R * 0.01f;
}

void EKF::normalizeQuaternion() {
    float norm = sqrt(x(0)*x(0) + x(1)*x(1) + x(2)*x(2) + x(3)*x(3));
    if (norm <= 1e-12f) return;
    for (int i = 0; i < 4; i++) x(i) /= norm; 
}