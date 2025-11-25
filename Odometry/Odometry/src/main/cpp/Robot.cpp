#include "Robot.h"
#include <frc/MathUtil.h>
#include <cstdio>
#include "math/MathUtils.h"
#include "hardware/SparkFlexMotorController.h"
#include "hardware/SparkMaxMotorController.h"

enum class MotorType {Max, Flex};

struct MotorConfig {
    int driveID;
    MotorType driveType;
    int steerID;
    MotorType steerType;
};

const MotorConfig swerveMotorConfigs[4] = {
    {10, MotorType::Flex,  11, MotorType::Max},   // front-left
    {16, MotorType::Flex, 17, MotorType::Max},  // front-right
    {12, MotorType::Flex,  13, MotorType::Max},   // back-left
    {14, MotorType::Flex, 15, MotorType::Max},  // back-right
};

Robot::Robot() 
    : kinematics(nullptr), odometry(nullptr) 
{
    // Nothing else here; RobotInit() does the real setup
}

Robot::~Robot() {}

void Robot::RobotInit() {
    // Define wheel offsets first
    printf("RobotInit begin\n"); fflush(stdout);
    std::array<Vector2D, SwerveConstants::NUM_WHEELS> wheelOffsets = {
        Vector2D(0.2575,  0.2575),   // front-left
        Vector2D(0.2575, -0.2575),   // front-right
        Vector2D(-0.2575, 0.2575),   // back-left
        Vector2D(-0.2575, -0.2575)   // back-right
    };

    printf("building modules\n"); fflush(stdout);
    // Build your swerve modules with the new polymorphic controllers
    for (size_t i = 0; i < SwerveConstants::NUM_WHEELS; ++i) {
        const auto& cfg = swerveMotorConfigs[i];

        std::unique_ptr<IMotorController> driveMotor;
        std::unique_ptr<IMotorController> steerMotor;

        // Create drive motor
        if (cfg.driveType == MotorType::Max)
            driveMotor = std::make_unique<SparkMaxMotorController>(cfg.driveID, rev::spark::SparkMax::MotorType::kBrushless);
        else
            driveMotor = std::make_unique<SparkFlexMotorController>(cfg.driveID, rev::spark::SparkFlex::MotorType::kBrushless);

        // Create steer motor
        if (cfg.steerType == MotorType::Max)
            steerMotor = std::make_unique<SparkMaxMotorController>(cfg.steerID, rev::spark::SparkMax::MotorType::kBrushless);
        else
            steerMotor = std::make_unique<SparkFlexMotorController>(cfg.steerID, rev::spark::SparkFlex::MotorType::kBrushless);

        modules[i] = std::make_shared<SwerveModule>(
            std::move(driveMotor), std::move(steerMotor),
            wheelOffsets[i], 0.0375, 5.50, 0.0);
    }
    printf("Modules build\n"); fflush(stdout);

    // Initialize kinematics
    kinematics = std::make_unique<SwerveDriveKinematics>(wheelOffsets);
    kinematics->toggleFieldRelativeControl(false);
    printf("Kinematics initialized\n"); fflush(stdout);

    

    // Reset drive encoders only
    for (size_t i = 0; i < SwerveConstants::NUM_WHEELS; i++) {
        modules[i]->reset();
    }
    printf("Reset drive encoders\n"); fflush(stdout);

        // Print initial absolute angles
    for (size_t i = 0; i < SwerveConstants::NUM_WHEELS; i++) {
        printf("Wheel %zu initial absolute angle=%.3f rad\n", i, modules[i]->getSteerAngle());
    }

    // Initialize odometry
    //odometry = std::make_unique<Odometry>(modules, &gyro);
    // odometry = std::make_unique<Odometry>(std::move(modules), std::make_unique<GyroSensor>());
    odometry = std::make_unique<Odometry>(modules, std::make_unique<GyroSensor>());
    odometry->resetPose(0,0,0);
    printf("Odometry initialized\n"); fflush(stdout);

    gyro.reset();



    printf("RobotInit complete.\n");
}


void Robot::TeleopPeriodic() {
    // printf("FL absolute encoder: %.3f rad\n", modules[0]->getSteerAngle());
    // printf("FR absolute encoder: %.3f rad\n", modules[1]->getSteerAngle());
    // printf("BL absolute encoder: %.3f rad\n", modules[2]->getSteerAngle());
    // printf("BR absolute encoder: %.3f rad\n", modules[3]->getSteerAngle());

    // --- 1. Read joystick axes with deadband ---
    double fwd = -frc::ApplyDeadband(joystick.GetRawAxis(1), 0.05); // forward/back
    double str = frc::ApplyDeadband(joystick.GetRawAxis(0), 0.05);  // strafe
    double rot = frc::ApplyDeadband(joystick.GetRawAxis(4), 0.05);  // rotation

    // --- 2. Scale to max speeds ---
    fwd *= OperatorConstants::MAX_FORWARD_SPEED_MPS;
    str *= OperatorConstants::MAX_SIDE_SPEED_MPS;
    rot *= OperatorConstants::MAX_ANGULAR_SPEED_RPS;

    // --- 3. Build chassis motion command ---
    ChassisState chassisState(Vector2D(fwd, str), rot);

    // --- 4. Convert chassis command to wheel states ---
    auto wheelStates = kinematics->toWheelStates(chassisState, odometry->getPose());

    // --- 5. Apply wheel states to modules using absolute steering encoder ---
    for (size_t i = 0; i < SwerveConstants::NUM_WHEELS; i++) {
        printf("Teleop: Setting wheel %zu: speed=%.2f, angle=%.2f\n", i, wheelStates[i].speed, wheelStates[i].angle);
        modules[i]->setDesiredState(wheelStates[i]);
    }

    // --- 6. Update odometry based on current wheel states and gyro ---
    odometry->update();

    // --- 7. Debug print: chassis state, wheel states, robot pose ---
    Pose currentPose = odometry->getPose();
    printf("Robot Pose: x=%.2f, y=%.2f, heading=%.2f°\n",
           currentPose.position.x,
           currentPose.position.y,
           currentPose.heading * 180.0 / OperatorConstants::PI);

    for (size_t i = 0; i < SwerveConstants::NUM_WHEELS; i++) {
        auto ws = modules[i]->getCurrentState();
        printf("Wheel %zu: speed=%.2f m/s, angle=%.3f rad (%.1f°), driveDist=%.2f m\n",
               i, ws.speed, ws.angle, ws.angle * 180.0 / OperatorConstants::PI,
               modules[i]->getDriveDistance());
    }

    // --- 8. Optional: print raw joystick for debugging ---
    printf("Joystick raw axes: fwd=%.2f str=%.2f rot=%.2f\n", 
           joystick.GetRawAxis(1), joystick.GetRawAxis(0), joystick.GetRawAxis(4));
}

void Robot::DisabledInit(){
    for (size_t i = 0; i < SwerveConstants::NUM_WHEELS; i++) {
        modules[i]->stop();  // call stop() on each module
    }
    printf("Robot disabled: all motors stopped.\n");
}
