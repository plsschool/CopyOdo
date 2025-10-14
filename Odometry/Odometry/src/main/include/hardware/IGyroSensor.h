#pragma once

// For angular orientation sensors (e.g., navX gyro)
class IGyroSensor {
public:
    virtual ~IGyroSensor() = default;
    virtual double getHeading() const = 0;    // in radians
    virtual double getRate() const = 0;       // in rad/s
    virtual void reset() = 0;
};