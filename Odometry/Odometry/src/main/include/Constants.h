// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;

    constexpr double PI = 3.14159265358979323846;

    constexpr double TRACK_WIDTH = 0.515;
    constexpr double WHEEL_BASE  = 0.515;

    // Max speeds
    constexpr double MAX_SPEED = 0.5; // m/s (same as Java's limit for safety)

    // Steer offsets (from the working Java code)
    constexpr double FRONT_LEFT_OFFSET  = M_PI;
    constexpr double FRONT_RIGHT_OFFSET = 0.0;
    constexpr double BACK_LEFT_OFFSET   = M_PI;
    constexpr double BACK_RIGHT_OFFSET  = 0.0;

    constexpr double DRIVE_DEADBAND = 0.2;
    constexpr double STEER_MOTOR_KP = 0.25;

}  // namespace OperatorConstants
