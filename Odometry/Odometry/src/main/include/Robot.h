// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "hardware/EncoderSensor.h"
#include "hardware/GyroSensor.h"
#include "odometry/Odometry.h"
#include "swerve/SwerveDriveKinematics.h"
#include "swerve/SwerveModule.h"
#include "odometry/Pose.h"
#include "Constants.h"

#include <frc/Joystick.h>
#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include <memory>

#include "RobotContainer.h"


class Robot : public frc::TimedRobot {
 public:
  Robot();
  ~Robot() override;
  // void RobotPeriodic() override;
  // void DisabledInit() override;
  // void DisabledPeriodic() override;
  // void AutonomousInit() override;
  // void AutonomousPeriodic() override;
  // void TeleopInit() override;

  // void TestPeriodic() override;
  // void SimulationInit() override;
  // void SimulationPeriodic() override;
  
  void TeleopPeriodic() override;
  void RobotInit() override;
  void DisabledInit() override;


 private:
  // Have it empty by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  std::optional<frc2::CommandPtr> m_autonomousCommand;
  frc::Joystick joystick{0};

  std::array<std::shared_ptr<SwerveModule>, SwerveConstants::NUM_WHEELS> modules;
  std::unique_ptr<SwerveDriveKinematics> kinematics;
  Pose pose;
  std::unique_ptr<GyroSensor> gyro;

  std::unique_ptr<Odometry> odometry;

  
    double lastHeading = 0.0;

    // Speed limits

    double lastDistances[SwerveConstants::NUM_WHEELS] = {0};

};
