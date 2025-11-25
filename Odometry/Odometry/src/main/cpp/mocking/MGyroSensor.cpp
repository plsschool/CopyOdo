#include "mocking/MGyroSensor.h"

MockGyroSensor::MockGyroSensor(double initialAngle, double initialRate) : angle(initialAngle), rate(initialRate) {}

double MockGyroSensor::getHeading() const {
    return angle;
}

// Signature should return double and be const to match the real IGyroSensor interface
double MockGyroSensor::getRate() const {
    return rate;
}

void MockGyroSensor::reset(){}