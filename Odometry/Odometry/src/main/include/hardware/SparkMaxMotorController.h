#include <rev/SparkMax.h>
#include "Interfaces/IMotorController.h"

class SparkMaxMotorController : public IMotorController {
    rev::spark::SparkMax motor;
    rev::spark::SparkRelativeEncoder encoder;
public:
    SparkMaxMotorController(int canID, rev::spark::SparkMax::MotorType type)
    : motor(canID, type), encoder(motor.GetEncoder()) {}

    void Set(double value) override { motor.Set(value); }
    void SetPosition(double position) override { encoder.SetPosition(position); }
    double GetPosition() const override { return encoder.GetPosition(); }
    double GetVelocity() const override { return encoder.GetVelocity(); }
    int GetDeviceId() const override { return motor.GetDeviceId(); }
};