#include <rev/SparkMax.h>
#include "Interfaces/IMotorController.h"

class SparkMaxMotorController : public IMotorController {
    rev::spark::SparkMax motor;
    rev::spark::SparkRelativeEncoder encoder;
public:
    SparkMaxMotorController(int canID, rev::spark::SparkMax::MotorType type)
    : motor(canID, type), encoder(motor.GetEncoder()) {}

    void Set(double value) override { 
        printf("SparkMaxMotorController %d: Set(%f)\n", motor.GetDeviceId(), value);
        motor.Set(value); 
    }
    void SetPosition(double position) override {
        printf("SparkMaxMotorController %d: SetPosition(%f)\n", motor.GetDeviceId(), position);
        encoder.SetPosition(position);
    }
    double GetPosition() const override {
        double pos = encoder.GetPosition();
        printf("SparkMaxMotorController %d: GetPosition() = %f\n", motor.GetDeviceId(), pos);
        return pos;
    }
    double GetVelocity() const override {
        double vel = encoder.GetVelocity();
        printf("SparkMaxMotorController %d: GetVelocity() = %f\n", motor.GetDeviceId(), vel);
        return vel;
    }

    int GetDeviceId() const override { return motor.GetDeviceId(); }
};