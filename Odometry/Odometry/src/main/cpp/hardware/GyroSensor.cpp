#include "hardware/GyroSensor.h"
#include <numbers>


GyroSensor::GyroSensor() : ahrs(studica::AHRS::kMXP_SPI, studica::AHRS::k50Hz)
{
    ahrs.Reset();
}

// double GyroSensor::getValue() const {
//     // Convert degrees to radians
//     //return ahrs.GetAngle() * std::numbers::pi / 180.0;
//     // Convert to radians and normalize to [-π, π]
//     double angleRad = ahrs.GetAngle() * OperatorConstants::PI / 180.0;
//     angleRad = std::fmod(angleRad, 2.0 * OperatorConstants::PI);
//     if (angleRad > OperatorConstants::PI)
//         angleRad -= 2.0 * OperatorConstants::PI;
//     else if (angleRad < -OperatorConstants::PI)
//         angleRad += 2.0 * OperatorConstants::PI;
//     return angleRad;
// }

double GyroSensor::getValue() const {
    // Return unbounded heading in radians
    return ahrs.GetAngle() * OperatorConstants::PI / 180.0;
}

double GyroSensor::getHeading() const{
    return getValue();
}

void GyroSensor::reset() {
    ahrs.Reset();
}

double GyroSensor::getRate() const {
    // Return rotation rate in radians per second
    return ahrs.GetRate() * OperatorConstants::PI / 180.0;
}
