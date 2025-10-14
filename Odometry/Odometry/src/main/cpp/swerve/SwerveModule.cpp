#include "swerve/SwerveModule.h"
#include <cmath>
#include <math.h>

SwerveModule::SwerveModule(int driveID, int steerID, double wheelRadius_, double gearRatio_, double steerOffset_)
    : driveMotor(driveID, rev::spark::SparkMax::MotorType::kBrushless),
      steerMotor(steerID, rev::spark::SparkMax::MotorType::kBrushless),
      driveEncoder(driveMotor.GetEncoder()),  // reference to concrete encoder
      steerEncoder(steerMotor, wheelRadius_, gearRatio_, true),               // if using integrated encoder for steering
      wheelRadius(wheelRadius_),
      gearRatio(gearRatio_), steerOffset(steerOffset_)
{
    driveEncoder.SetPosition(0);
    steerEncoder.reset();
}

// void SwerveModule::setDesiredState(const WheelModuleState& state) {
//     // --- Drive motor ---
//     double percentOutput = state.speed / 1.5; // normalize prev 3.0
//     //double rpm = (state.speed / (2.0 * PI * wheelRadius)) * 60.0 * gearRatio;
//     driveMotor.Set(std::clamp(percentOutput, -1.0, 1.0));
//     // If using velocity closed loop:
//     // double wheelRPS = state.speed / (2.0 * PI * wheelRadius);
//     // double motorRPM = wheelRPS * 60.0 * gearRatio;
//     // drivePID.SetReference(motorRPM, rev::CANSparkMax::ControlType::kVelocity);

//     // --- Steer motor ---
//     double currentAngle = steerEncoder.getValue(); // in radians
//     double targetAngle = optimizeSteerAngle(currentAngle, state.angle);
//     double error = MathUtils::normalizeAngle(targetAngle - currentAngle);

//     double kP = 0.5; // might need tuning between 0.1 and 1.0
//     double output = std::clamp(kP * error, -1.0, 1.0);

//     steerMotor.Set(output); // may need conversion to motor units
// }

// void SwerveModule::setDesiredState(const WheelModuleState& state) {
//     static int loopCount = 0;

//     // --- Drive motor (percent output) ---
//     double percentOutput = std::clamp(state.speed / 1.5, -1.0, 1.0);
//     driveMotor.Set(percentOutput);
//     //printf("Wheel %d speed cmd: %.2f\n", driveMotor.GetDeviceId(), percentOutput);


//     // --- Steer motor (angle control) ---
//     //double currentAngle = steerMotor.GetEncoder().GetPosition() * 2.0 * PI * gearRatio; // convert rotations → radians
//     //double currentAngle = steerMotor.GetEncoder().GetPosition() * (2.0 * PI / gearRatio) - steerOffset;

//     // Steer encoder returns rotations of the motor
//     double currentAngle = steerMotor.GetEncoder().GetPosition() * 2.0 * PI / gearRatio;
//     currentAngle = MathUtils::normalizeAngle(currentAngle + steerOffset);

//     double targetAngle = optimizeSteerAngle(currentAngle, state.angle);
//     double error = MathUtils::normalizeAngle(targetAngle - currentAngle);

//     double kP = 0.1; // tune for responsiveness
//     double output = std::clamp(kP * error, -1.0, 1.0);
//     steerMotor.Set(output);

//     // Debug
//     printf("Wheel %d: target=%.3f rad, current=%.3f rad, error=%.3f rad\n",
//        driveMotor.GetDeviceId(), targetAngle, currentAngle, error);

//     // --- Debug print every ~10 cycles (≈200ms) ---
//     if (++loopCount % 10 == 0) {
//         printf("Module[%d] drive=%.2f steerOut=%.2f target=%.2f curr=%.2f err=%.2f\n",
//                driveMotor.GetDeviceId(), percentOutput, output,
//                targetAngle, currentAngle, error);
//     }
// }
void SwerveModule::setDesiredState(const WheelModuleState& state) {
    static int loopCount = 0;

    // --- Drive motor (percent output) ---
    double percentOutput = std::clamp(state.speed / MAX_SPEED, -1.0, 1.0);
    driveMotor.Set(percentOutput);

    // --- Steer motor (angle control) ---
    // Convert encoder rotations to radians
    double currentAngle = steerMotor.GetEncoder().GetPosition() * (2.0 * PI / steerGearRatio);

    // Normalize current angle to [-PI, PI]
    currentAngle = MathUtils::normalizeAngle(currentAngle);

    // Optimize target angle to minimize rotation
    double targetAngle = optimizeSteerAngle(currentAngle, state.angle);

    // Compute error and apply simple P control
    double error = MathUtils::normalizeAngle(targetAngle - currentAngle);

    double kP = 0.2; // may need tuning
    double output = std::clamp(kP * error, -1.0, 1.0);
    steerMotor.Set(output);

    // --- Debug print every 10 cycles ---
    if (++loopCount % 10 == 0) {
        printf("Module[%d] drive=%.2f steerOut=%.2f target=%.2f curr=%.2f err=%.2f\n",
               driveMotor.GetDeviceId(), percentOutput, output,
               targetAngle, currentAngle, error);
    }
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

double SwerveModule::getDriveDistance() const {
    return driveEncoder.GetPosition() * (2.0 * PI * wheelRadius / gearRatio);
}

void SwerveModule::stop(){
    driveMotor.Set(0);
    steerMotor.Set(0);
}

