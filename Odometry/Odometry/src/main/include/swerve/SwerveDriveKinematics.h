// #pragma once
// #include <array>
// #include "WheelModuleState.h"
// #include "ChassisState.h"
// #include "Math/Vector2D.h"
// #include "odometry/Pose.h"


// class SwerveDriveKinematics {
// public:
//     static constexpr int NUM_WHEELS = 4;
//     static constexpr int MAX_SPEED = 5;

//     std::array<Vector2D, NUM_WHEELS> wheelOffsets;

//     explicit SwerveDriveKinematics(const std::array<Vector2D, NUM_WHEELS>& offsets);

//     std::array<WheelModuleState, NUM_WHEELS> toWheelStates(const ChassisState& chassisState, const Pose& pose);
//     ChassisState toChassisState(const std::array<WheelModuleState, NUM_WHEELS>& wheelStates);
    
//     void toggleFieldRelativeControl(bool enabled);
//     bool isFieldRelative() const;

// private:
//     bool fieldRelative = true;
//     void normalizeWheelSpeeds(std::array<WheelModuleState, NUM_WHEELS>& states, double maxSpeed);
// };
#pragma once

#include <array>
#include "swerve/ChassisState.h"
#include "swerve/WheelModuleState.h"
#include "math/Vector2D.h"
#include "odometry/Pose.h"

class SwerveDriveKinematics {
public:
    static constexpr int NUM_WHEELS = 4;
    static constexpr double MAX_SPEED = 2.0; // adjust to your robot’s max speed (m/s)

    explicit SwerveDriveKinematics(const std::array<Vector2D, NUM_WHEELS>& offsets);

    std::array<WheelModuleState, NUM_WHEELS> toWheelStates(
        const ChassisState& chassisState, 
        const Pose& pose
    );

    ChassisState toChassisState(
        const std::array<WheelModuleState, NUM_WHEELS>& wheelStates
    );

    void normalizeWheelSpeeds(
        std::array<WheelModuleState, NUM_WHEELS>& states, 
        double maxSpeed
    );

    void toggleFieldRelativeControl(bool enabled);
    bool isFieldRelative() const;

private:
    bool fieldRelative = false;
    std::array<Vector2D, NUM_WHEELS> wheelOffsets;
};
