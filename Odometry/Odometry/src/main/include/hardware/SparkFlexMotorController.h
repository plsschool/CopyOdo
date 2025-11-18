#include <rev/SparkFlex.h>
#include "Interfaces/IMotorController.h"

class SparkFlexMotorController : public IMotorController {
    rev::spark::SparkFlex motor;
    rev::spark::SparkRelativeEncoder encoder;

public:
    SparkFlexMotorController(int canID, rev::spark::SparkFlex::MotorType type)
    : motor(canID, type), encoder(motor.GetEncoder()) {}

    void Set(double value) override { motor.Set(value); }
    void SetPosition(double position) override { encoder.SetPosition(position); }
    double GetPosition() const override { return encoder.GetPosition(); }
    double GetVelocity() const override { return encoder.GetVelocity(); }
    int GetDeviceId() const override { return motor.GetDeviceId(); }
};