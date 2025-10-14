#include "Robot.h"
#include <frc/MathUtil.h>
#include <cstdio>

Robot::Robot() 
    : kinematics(nullptr), odometry(nullptr) 
{
    // Nothing else here; RobotInit() does the real setup
}

Robot::~Robot() {
    for (auto& module : modules) {
        delete module;
    }
    delete kinematics;
    delete odometry;
}

void Robot::RobotInit() {
    modules = {
        new SwerveModule(10, 11, 0.0375, 5.50, 0.00), // front-left
        modules[1] = new SwerveModule(16, 17, 0.0375, 5.50, -0.00), // front-right
        modules[2] = new SwerveModule(12, 13, 0.0375, 5.50, -0.00), // back-lef
        modules[3] = new SwerveModule(14, 15, 0.0375, 5.50, 0.00) // back-right
    };
    // Example wheel offsets (meters)
    std::array<Vector2D, NUM_WHEELS> wheelOffsets = {
        Vector2D(0.2575,  0.2575),   // front-left
        Vector2D(0.2575, -0.2575),   // front-right
        Vector2D(-0.2575, 0.2575),   // back-left
        Vector2D(-0.2575, -0.2575)   // back-right
    };

    // Initialize kinematics
    kinematics = new SwerveDriveKinematics(wheelOffsets);
    kinematics->toggleFieldRelativeControl(false);
    odometry = new Odometry(modules, &gyro);


    odometry->resetPose(0,0,0);
    
    // Reset getDriveDistance
    for (int i = 0; i < NUM_WHEELS; i++) {
        modules[i]->reset();  // resets drive encoder and steer encoder
    }
    gyro.reset();
    printf("RobotInit complete.\n");
    // Create swerve modules (IDs depend on wiring)
    // modules[0] = new SwerveModule(10, 11, 0.0375, 5.50); // front-left
    // modules[1] = new SwerveModule(16, 17, 0.0375, 5.50); // front-right
    // modules[2] = new SwerveModule(12, 13, 0.0375, 5.50); // back-left
    // modules[3] = new SwerveModule(14, 15, 0.0375, 5.50); // back-right
    
    // Reset odometry
    // pose.resetPose(0.0, 0.0, 0.0);
    // lastHeading = gyro.getHeading();
}

void Robot::TeleopPeriodic() {
    // --- 1. Read joystick axes with deadband ---
    double fwd = -frc::ApplyDeadband(joystick.GetRawAxis(1), 0.05); // forward/back
    double str = frc::ApplyDeadband(joystick.GetRawAxis(0), 0.05);  // strafe
    double rot = frc::ApplyDeadband(joystick.GetRawAxis(4), 0.05);  // rotation

    // --- 2. Scale joystick input to physical units ---

    // if (fabs(fwd) < 0.001 && fabs(str) < 0.001 && fabs(rot) < 0.001) {
    //     auto wheelStates = kinematics->toWheelStates(ChassisState(Vector2D(0,0), 0), odometry->getPose());
    //     for (int i = 0; i < NUM_WHEELS; i++) {
    //         printf("Init wheel[%d] targetAngle=%.2f rad\n", i, wheelStates[i].angle);
    //     }
    // }

    fwd *= maxForwardSpeed;
    str *= maxSideSpeed;
    rot *= maxAngularSpeed;
    
    if (fabs(fwd) < 0.001 && fabs(str) < 0.001 && fabs(rot) < 0.001) {
        auto wheelStates = kinematics->toWheelStates(ChassisState(Vector2D(0,0), 0), odometry->getPose());
        for (int i = 0; i < NUM_WHEELS; i++) {
            printf("Init wheel[%d] targetAngle=%.2f rad\n", i, wheelStates[i].angle);
        }
    }
    // --- 3. build chassis motion commands ---
    ChassisState chassisState(Vector2D(fwd, str), rot);
    printf("Joystick fwd=%.2f str=%.2f rot=%.2f\n", fwd, str, rot);
    // --- 4. Convert to wheel states ---
    auto wheelStates = kinematics->toWheelStates(chassisState, odometry->getPose());

    // --- 5. Apply to modules ---
    for (int i = 0; i < NUM_WHEELS; i++) {
        modules[i]->setDesiredState(wheelStates[i]);
    }

    for (int i=0;i<4;i++){
    printf("Steer encoder %d raw=%.3f rad\n", i, modules[i]->getCurrentState().angle);
}


    // --- 6. Update the odometry
    odometry->update();

    // --- 7. Debug: print robot pose ---
    Pose currentPose = odometry->getPose();
    printf("Pose: x=%.2f, y=%.2f, heading=%.2f°\n",
           currentPose.position.x,
           currentPose.position.y,
           currentPose.heading * 180.0 / PI);
}

void Robot::DisabledInit(){
    for (int i = 0; i < NUM_WHEELS; i++) {
        modules[i]->stop();  // call stop() on each module
    }
    printf("Robot disabled: all motors stopped.\n");
}

