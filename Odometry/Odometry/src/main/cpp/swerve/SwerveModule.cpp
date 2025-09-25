#include "swerve/SwerveModule.h"
#include <cmath>
#include <math.h>

SwerveModule::SwerveModule(int driveID, int steerID, double wheelRadius_, double gearRatio_)
    : driveMotor(driveID, rev::spark::SparkMax::MotorType::kBrushless),
      steerMotor(steerID, rev::spark::SparkMax::MotorType::kBrushless),
      driveEncoder(driveMotor.GetEncoder()),  // reference to concrete encoder
      steerEncoder(steerMotor, wheelRadius_, gearRatio_),               // if using integrated encoder for steering
      wheelRadius(wheelRadius_),
      gearRatio(gearRatio_)
{
    driveEncoder.SetPosition(0);
    steerEncoder.reset();
}

void SwerveModule::setDesiredState(const WheelModuleState& state) {
    // --- Drive motor ---
    double rpm = (state.speed / (2.0 * PI * wheelRadius)) * 60.0 * gearRatio;
    driveMotor.Set(rpm);

    // --- Steer motor ---
    double currentAngle = steerEncoder.getValue(); // in radians
    double targetAngle = optimizeSteerAngle(currentAngle, state.angle);
    steerMotor.Set(targetAngle); // may need conversion to motor units
}

WheelModuleState SwerveModule::getCurrentState() const {
    WheelModuleState state;
    state.speed = (driveEncoder.GetVelocity() / 60.0) * 2.0 * PI * wheelRadius / gearRatio; // m/s
    state.angle = steerEncoder.getValue(); // radians
    return state;
}

void SwerveModule::reset() {
    driveEncoder.SetPosition(0);
    steerEncoder.reset();
}

double SwerveModule::optimizeSteerAngle(double currentAngle, double targetAngle) const {
    double delta = MathUtils::normalizeAngle(targetAngle - currentAngle);
    return currentAngle + delta;
}
