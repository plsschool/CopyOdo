#include "swerve/SwerveModule.h"

// SwerveModule::SwerveModule(int driveID, int steerID, Vector2D wheelOffset_, double wheelRadius_, double gearRatio_, double steerOffset_)
//     : driveMotor(driveID, rev::spark::SparkMax::MotorType::kBrushless),
//       steerMotor(steerID, rev::spark::SparkMax::MotorType::kBrushless),
//       driveEncoder(driveMotor.GetEncoder()),
//       steerEncoder(steerMotor.GetAbsoluteEncoder()),
//       wheelOffset(wheelOffset_),
//       wheelRadius(wheelRadius_),
//       gearRatio(gearRatio_),
//       steerOffset(steerOffset_)
// {
//     driveEncoder.SetPosition(0);
//     // Do NOT reset absolute encoder; just apply offset
// }

SwerveModule::SwerveModule(std::unique_ptr<IMotorController> driveMotor_, std::unique_ptr<IMotorController> steerMotor_, Vector2D wheelOffset_, double wheelRadius_, double gearRatio_, double steerOffset_)
    : driveMotor(std::move(driveMotor_)),
      steerMotor(std::move(steerMotor_)),
      wheelOffset(wheelOffset_),
      wheelRadius(wheelRadius_),
      gearRatio(gearRatio_),
      steerOffset(steerOffset_)
{
    // Reset drive encoder's position to 0 via the interface
    driveMotor->SetPosition(0.0);
    // If your IMotorController supports absolute encoder reset/offset, use that here
    // Otherwise, handle absolute encoder offset in your logic
}

void SwerveModule::setDesiredState(const WheelModuleState& state) {
    static int loopCount = 0;

    double currentAngle = MathUtils::normalizeAngle(steerMotor->GetPosition() - steerOffset);

    // Compute raw delta from requested angle to current angle (signed, [-PI,PI])
    double rawDelta = MathUtils::normalizeAngle(state.angle - currentAngle);

    // Drive motor percent output (before possible inversion)
    double percentOutput = std::clamp(state.speed / SwerveConstants::MAX_WHEEL_SPEED_MPS, -1.0, 1.0);

    // If rawDelta would require rotating more than 90 degrees, we prefer to flip wheel
    // direction and rotate the steering by 180 degrees. In that case invert drive sign
    bool shouldInvertDrive = std::fabs(rawDelta) > (OperatorConstants::PI / 2.0);
    if (shouldInvertDrive) {
        percentOutput = -percentOutput;
    }

   // Apply a small deadband so tiny commands don't spin motors
    const double DRIVE_DEADBAND = 0.02; // 2% output threshold (tune if needed)
    if (std::fabs(percentOutput) < DRIVE_DEADBAND) {
        percentOutput = 0.0;
    }
    printf("SwerveModule: setDesiredState - speed: %.2f, angle: %.2f (rad)\n", state.speed, state.angle);
    driveMotor->Set(percentOutput);

    // Steering motor control using absolute encoder
    double targetAngle = optimizeSteerAngle(currentAngle, state.angle);
    double error = MathUtils::normalizeAngle(targetAngle - currentAngle);

    double output = std::clamp(PIDGains::STEER_kP* error, -1.0, 1.0);
    printf("SwerveModule: setDesiredState - speed: %.2f, angle: %.2f (rad)\n", state.speed, state.angle);
    steerMotor->Set(output);

    if (++loopCount % 10 == 0) {
        printf("Module[%d] drive=%.3f steerOut=%.3f target=%.3f curr=%.3f rawDelta=%.3f invert=%d\n",
               driveMotor->GetDeviceId(), percentOutput, output,
               targetAngle, currentAngle, rawDelta, shouldInvertDrive ? 1 : 0);
    }
}

WheelModuleState SwerveModule::getCurrentState() const {
    WheelModuleState state;
    state.speed = (driveMotor->GetVelocity() / 60.0) * 2.0 * OperatorConstants::PI * wheelRadius / gearRatio;
    state.angle = MathUtils::normalizeAngle(steerMotor->GetPosition() - steerOffset);
    return state;
}

void SwerveModule::reset() {
    driveMotor->SetPosition(0);
    // Do NOT reset absolute encoder
}

double SwerveModule::getDriveDistance() const {
    return driveMotor->GetPosition() * 2.0 * OperatorConstants::PI * wheelRadius / gearRatio;
}

double SwerveModule::getSteerAngle() const {
    return MathUtils::normalizeAngle(steerMotor->GetPosition() - steerOffset);
}

void SwerveModule::stop() {
    driveMotor->Set(0);
    steerMotor->Set(0);
}

double SwerveModule::optimizeSteerAngle(double currentAngle, double targetAngle) const {
    double delta = MathUtils::normalizeAngle(targetAngle - currentAngle);

    if (delta > OperatorConstants::PI/2) delta -= OperatorConstants::PI;
    if (delta < -OperatorConstants::PI/2) delta += OperatorConstants::PI;

    return MathUtils::normalizeAngle(currentAngle + delta);
}
