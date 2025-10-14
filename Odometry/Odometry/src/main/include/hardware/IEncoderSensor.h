#pragma once
// For linear position/velocity sensors (e.g., wheel encoders)
class IEncoderSensor {
public:
    virtual ~IEncoderSensor() = default;
    //virtual double getDistance() const = 0;   // in meters
    //virtual double getVelocity() const = 0;   // in m/s
    virtual void reset() = 0;
};