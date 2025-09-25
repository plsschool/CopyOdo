#pragma once
#include "Sensor.h"
#include <studica/AHRS.h>
#include <frc/SPI.h>

class GyroSensor : public Sensor{
public:
    GyroSensor();
    double getValue() const override;  // heading in radians
    double getHeading();
    void reset() override;
private:
    mutable studica::AHRS ahrs;  // navX gyro
};

