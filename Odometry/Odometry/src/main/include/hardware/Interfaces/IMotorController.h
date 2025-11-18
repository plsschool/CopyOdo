class IMotorController {
public:
    virtual void Set(double value) = 0; // -1.0 to 1.0 power control
    virtual void SetPosition(double position) = 0; // for steering, if needed
    virtual double GetPosition() const = 0; // encoder position
    virtual double GetVelocity() const = 0; // encoder velocity
    virtual int GetDeviceId() const = 0; // for debug
    virtual ~IMotorController() = default;
};