#include "WheelOdometry.h"
#include <Arduino.h>

WheelOdometry::WheelOdometry(TwoWire *i2cBus)
    : i2cBus(i2cBus), encoder(i2cBus), previousAngle(0.0f), totalAngle(0.0f) {}

void WheelOdometry::update() {
    // Read the current angle from the AS5600 encoder
    float currentAngle = encoder.rawAngle() * (360.0f / 4096.0f);

    // Calculate the difference between the current and previous angle
    float angleDifference = currentAngle - previousAngle;

    // Handle overflow
    if (angleDifference > 180.0f) {
        angleDifference -= 360.0f;
    } else if (angleDifference < -180.0f) {
        angleDifference += 360.0f;
    }

    // Accumulate the total angle
    totalAngle += angleDifference;
    previousAngle = currentAngle;
}

float WheelOdometry::getOdometry() const {
    // Convert total angle to radians and return
    return totalAngle * (PI / 180.0f);
}

void WheelOdometry::reset() {
    // Reset odometry readings
    previousAngle = 0.0f;
    totalAngle = 0.0f;
}
