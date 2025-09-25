#include "Robot.h"
#include <frc/MathUtil.h>
#include <cstdio>

Robot::Robot() 
    : kinematics(nullptr), lastHeading(0.0) 
{
    // Nothing else here; RobotInit() does the real setup
}

Robot::~Robot() {
    for (auto& module : modules) {
        delete module;
    }
    delete kinematics;
}

void Robot::RobotInit() {
    // Example wheel offsets (meters)
    std::array<Vector2D, NUM_WHEELS> wheelOffsets = {
        Vector2D(0.2575,  0.2575),   // front-left
        Vector2D(0.2575, -0.2575),   // front-right
        Vector2D(-0.2575, 0.2575),   // back-left
        Vector2D(-0.2575, -0.2575)   // back-right
    };

    // Initialize kinematics
    kinematics = new SwerveDriveKinematics(wheelOffsets);

    // Create swerve modules (IDs depend on wiring)
    modules[0] = new SwerveModule(10, 11, 0.0375, 5.50); // front-left
    modules[1] = new SwerveModule(17, 16, 0.0375, 5.50); // front-right
    modules[2] = new SwerveModule(12, 13, 0.0375, 5.50); // back-left
    modules[3] = new SwerveModule(14, 15, 0.0375, 5.50); // back-right

    // Reset odometry
    pose.resetPose(0.0, 0.0, 0.0);
    lastHeading = gyro.getHeading();
}

void Robot::TeleopPeriodic() {
    // --- 1. Read joystick axes with deadband ---
    double x = frc::ApplyDeadband(joystick.GetX(), 0.05);
    double y = frc::ApplyDeadband(joystick.GetY(), 0.05);
    double rot = frc::ApplyDeadband(joystick.GetZ(), 0.05);

    // --- 2. Scale joystick input to physical units ---
    Vector2D velocity(
        x * maxForwardSpeed,  // m/s
        y * maxSideSpeed      // m/s
    );
    double omega = rot * maxAngularSpeed; // rad/s

    // --- 3. Odometry update (Phase 1) ---
    double heading = gyro.getHeading();
    double deltaHeading = heading - lastHeading;

    double avgDistance = 0.0;
    for (auto& module : modules) {
        avgDistance += module->getCurrentState().speed * 0.02; // 20ms loop
    }
    avgDistance /= NUM_WHEELS;

    pose.updatePose(avgDistance, deltaHeading);
    lastHeading = heading;

    // Debug print
    printf("Pose: x=%.2f, y=%.2f, heading=%.2f deg\n", pose.position.x, pose.position.y, MathUtils::radToDeg(pose.getHeading()));

    // --- 4. Build chassis state (Phase 2) ---
    ChassisState chassisState(velocity, omega);

    // --- 5. Compute wheel states from chassis state ---
    auto wheelStates = kinematics->toWheelStates(chassisState, pose);

    // --- 6. Apply wheel states to each swerve module ---
    for (int i = 0; i < NUM_WHEELS; i++) {
        modules[i]->setDesiredState(wheelStates[i]);
    }
}
