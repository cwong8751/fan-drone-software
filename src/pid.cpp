#include "pid.h"
#include "config.h"

// TODO: cascaded PID control where inner is gyro-rate based and outer is gyro-position based.
//       in the future, we'll have to consider yaw rate. 
// TODO: we need to design, outline, and implement code to handle our transmitter connection and parse the 
//       data frames transmitted such that we can imbed its data into our pid control loop.
// NOTE: chris provided a library which should allow us to control our servos. logic: TRANSMITTER DATA -> PID CONTROL -> PWM SIGNAL -> SERVOS + MOTOR
// we'll probably have to incldue the flight state structure to grab our rates. this will involve grabbing the mutex which 
// will introduce an unknown amount of delay to our control loop. 
