#ifndef CF_H
#define CF_H

#include <Arduino.h>

class CF {
public:
    CF(float alpha = 0.98f);

    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

    float getRoll() const;
    float getPitch() const;

private:
    float alpha; // filter blending factor
    float roll;  // filtered roll angle 
    float pitch; // filtered pitch angle 
};

#endif // CF_H