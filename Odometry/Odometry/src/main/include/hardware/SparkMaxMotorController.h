#include <rev/SparkMax.h>
#include "Interfaces/IMotorController.h"

// Wrapper for CANSparkMax-like API that can return either the relative encoder
// or the absolute encoder position depending on how it was constructed.
class SparkMaxMotorController : public IMotorController {
    rev::spark::SparkMax motor;
    rev::spark::SparkRelativeEncoder relEncoder;
    // The SparkMax absolute encoder type used earlier in your repo; keep a member
    // so we can read the absolute position when requested.
    rev::spark::SparkAbsoluteEncoder absEncoder;
    bool useAbsoluteForPosition{false};

public:
    // useAbsoluteForPosition = true -> GetPosition() will return absolute encoder
    SparkMaxMotorController(int canID, rev::spark::SparkMax::MotorType type, bool useAbsoluteForPosition = false)
        : motor(canID, type),
          relEncoder(motor.GetEncoder()),
          absEncoder(motor.GetAbsoluteEncoder()),
          useAbsoluteForPosition(useAbsoluteForPosition) {}

    void Set(double value) override {
        //printf("SparkMaxMotorController %d: Set(%f)\n", motor.GetDeviceId(), value);
        motor.Set(value);
    }

    void SetPosition(double position) override {
        // Only set the relative encoder position; absolute encoders usually cannot/should not be reset here
        if (useAbsoluteForPosition) {
            //printf("SparkMaxMotorController %d: SetPosition(%f) called but this controller is configured to use ABSOLUTE encoder for GetPosition(); ignoring SetPosition\n", motor.GetDeviceId(), position);
            return;
        }
        //printf("SparkMaxMotorController %d: SetPosition(%f)\n", motor.GetDeviceId(), position);
        relEncoder.SetPosition(position);
    }

    double GetPosition() const override {
        double pos;
        if (useAbsoluteForPosition) {
            pos = absEncoder.GetPosition();
            //printf("SparkMaxMotorController %d: GetPosition() [ABS] = %f\n", motor.GetDeviceId(), pos);
        } else {
            pos = relEncoder.GetPosition();
            //printf("SparkMaxMotorController %d: GetPosition() [REL] = %f\n", motor.GetDeviceId(), pos);
        }
        return pos;
    }

    double GetVelocity() const override {
        double vel = relEncoder.GetVelocity();
        //printf("SparkMaxMotorController %d: GetVelocity() = %f\n", motor.GetDeviceId(), vel);
        return vel;
    }

    int GetDeviceId() const override { return motor.GetDeviceId(); }
};