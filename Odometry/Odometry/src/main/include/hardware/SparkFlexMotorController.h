#include <rev/SparkFlex.h>
#include "Interfaces/IMotorController.h"

class SparkFlexMotorController : public IMotorController {
    rev::spark::SparkFlex motor;
    rev::spark::SparkRelativeEncoder encoder;

public:
    SparkFlexMotorController(int canID, rev::spark::SparkFlex::MotorType type)
    : motor(canID, type), encoder(motor.GetEncoder()) {}

    void Set(double value) override { 
        printf("SparkFlexMotorController %d: Set(%f)\n", motor.GetDeviceId(), value);
        motor.Set(value); 
        }
        
    int GetDeviceId() const override { return motor.GetDeviceId(); }

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
};