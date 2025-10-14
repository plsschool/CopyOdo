#pragma once
#include "WheelModuleState.h"
#include "hardware/EncoderSensor.h"
#include "Math/MathUtils.h"

#include <rev/SparkMax.h>
#include <rev/RelativeEncoder.h>

class SwerveModule {
public:
    static constexpr double PI = 3.14159265358979323846;
    SwerveModule(int driveID, int steerID, double wheelRadius, double gearRatio = 1.0, double steerOffset = 0.00);

    void setDesiredState(const WheelModuleState& state);
    WheelModuleState getCurrentState() const;
    void reset();
    double getDriveDistance() const;
    void stop();

private:
    rev::spark::SparkMax driveMotor;
    rev::spark::SparkMax steerMotor;

    rev::RelativeEncoder& driveEncoder;  // store as reference
    EncoderSensor steerEncoder;          // optional: for precise steering

    double wheelRadius;  // meters
    double gearRatio;    // motor to wheel reduction
    double steerOffset;
    // Helper for angle optimization
    double optimizeSteerAngle(double currentAngle, double targetAngle) const;
};
