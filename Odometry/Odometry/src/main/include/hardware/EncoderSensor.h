#pragma once
#include <rev/SparkMax.h>
#include <rev/RelativeEncoder.h>
#include "IEncoderSensor.h"

class EncoderSensor : public IEncoderSensor {
public:
    static constexpr double PI = 3.14159265358979323846;
    // Constructor: motor object, wheel radius (m), optional gear ratio
    EncoderSensor(rev::spark::SparkMax& motor_, double wheelRadiusMeters, double gearRatio_ = 1.0, bool isSteering = false);

    double getValue() const;   // distance in meters
    void reset() override;

private:
    rev::RelativeEncoder& encoder;  // reference to NEO encoder
    double wheelRadius;        // meters
    double gearRatio;          // motor-to-wheel
    bool steeringMode;
};
