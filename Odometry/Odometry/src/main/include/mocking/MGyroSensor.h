#pragma once
#include "hardware/Interfaces/IGyroSensor.h"

class MockGyroSensor : public IGyroSensor {
private:
    double angle;
    double rate;
public:
    MockGyroSensor(double initialAngle, double initialRate);
    double getHeading() const override;
    double getRate() const override;
    void reset() override;
};