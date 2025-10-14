#include "hardware/GyroSensor.h"
#include <numbers>


GyroSensor::GyroSensor() : ahrs(studica::AHRS::kMXP_SPI, studica::AHRS::k50Hz)
{
    ahrs.Reset();
}

double GyroSensor::getValue() const {
    // Convert degrees to radians
    //return ahrs.GetAngle() * std::numbers::pi / 180.0;
    // Convert to radians and normalize to [-π, π]
    double angleRad = ahrs.GetAngle() * std::numbers::pi / 180.0;
    angleRad = std::fmod(angleRad, 2.0 * std::numbers::pi);
    if (angleRad > std::numbers::pi)
        angleRad -= 2.0 * std::numbers::pi;
    else if (angleRad < -std::numbers::pi)
        angleRad += 2.0 * std::numbers::pi;
    return angleRad;
}

double GyroSensor::getHeading() const{
    return getValue();
}

void GyroSensor::reset() {
    ahrs.Reset();
}

double GyroSensor::getRate() const {
    // Return rotation rate in radians per second
    return ahrs.GetRate() * std::numbers::pi / 180.0;
}
