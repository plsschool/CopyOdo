#include "hardware/EncoderSensor.h"
#include <cmath>

EncoderSensor::EncoderSensor(rev::spark::SparkMax& motor_, double wheelRadiusMeters, double gearRatio_)
    : encoder(motor_.GetEncoder()), wheelRadius(wheelRadiusMeters), gearRatio(gearRatio_) 
{
    encoder.SetPosition(0); // start counting from 0 rotations
}

double EncoderSensor::getValue() const {
    // rotations * 2 * pi * r * gearRatio = distance in meters
    return encoder.GetPosition() * 2.0 * PI * wheelRadius * gearRatio;
}

void EncoderSensor::reset() {
    encoder.SetPosition(0);
}
