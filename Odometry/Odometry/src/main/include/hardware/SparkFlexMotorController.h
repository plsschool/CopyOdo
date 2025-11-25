#include <rev/SparkFlex.h>
#include "Interfaces/IMotorController.h"

// Same approach for SparkFlex. Some API names differ between vendors but the
// pattern is identical: allow using the absolute encoder reading for steering.
class SparkFlexMotorController : public IMotorController {
    rev::spark::SparkFlex motor;
    rev::spark::SparkRelativeEncoder relEncoder;
    rev::spark::SparkAbsoluteEncoder absEncoder;
    bool useAbsoluteForPosition{false};

public:
    SparkFlexMotorController(int canID, rev::spark::SparkFlex::MotorType type, bool useAbsoluteForPosition = false)
        : motor(canID, type),
          relEncoder(motor.GetEncoder()),
          absEncoder(motor.GetAbsoluteEncoder()),
          useAbsoluteForPosition(useAbsoluteForPosition) {}

    void Set(double value) override {
        //printf("SparkFlexMotorController %d: Set(%f)\n", motor.GetDeviceId(), value);
        motor.Set(value);
    }

    void SetPosition(double position) override {
        if (useAbsoluteForPosition) {
            //printf("SparkFlexMotorController %d: SetPosition(%f) called but configured to use ABS encoder; ignoring SetPosition\n", motor.GetDeviceId(), position);
            return;
        }
        //printf("SparkFlexMotorController %d: SetPosition(%f)\n", motor.GetDeviceId(), position);
        relEncoder.SetPosition(position);
    }

    double GetPosition() const override {
        double pos;
        if (useAbsoluteForPosition) {
            pos = absEncoder.GetPosition();
            //printf("SparkFlexMotorController %d: GetPosition() [ABS] = %f\n", motor.GetDeviceId(), pos);
        } else {
            pos = relEncoder.GetPosition();
            //printf("SparkFlexMotorController %d: GetPosition() [REL] = %f\n", motor.GetDeviceId(), pos);
        }
        return pos;
    }

    double GetVelocity() const override {
        double vel = relEncoder.GetVelocity();
        //printf("SparkFlexMotorController %d: GetVelocity() = %f\n", motor.GetDeviceId(), vel);
        return vel;
    }

    int GetDeviceId() const override { return motor.GetDeviceId(); }
};