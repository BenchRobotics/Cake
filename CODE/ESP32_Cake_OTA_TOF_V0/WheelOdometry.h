#ifndef WHEEL_ODOMETRY_H
#define WHEEL_ODOMETRY_H

#include <Wire.h>
#include <AS5600.h>

class WheelOdometry {
public:
    WheelOdometry(TwoWire *i2cBus);
    void update(); // Updates odometry readings
    float getOdometry() const; // Returns the total angle in radians
    void reset(); // Resets the odometry

private:
    TwoWire *i2cBus;
    AS5600 encoder;
    float previousAngle;
    float totalAngle;
};

#endif // WHEEL_ODOMETRY_H
