#pragma once
#include "swerve/SwerveModule.h"
#include "hardware/Module.h"
#include "hardware/Interfaces/IGyroSensor.h"
#include "odometry/Pose.h"
#include "Constants.h"
#include <array>
#include <memory>

class Odometry : public Module {
public:
    Odometry(
        const std::array<std::shared_ptr<SwerveModule>, SwerveConstants::NUMBER_OF_MODULES>& modules_,
        IGyroSensor* gyro_
    );

    void update() override;
    Pose getPose() const;
    void resetPose(double x, double y, double heading);

private:
    // Store shared_ptrs here, not unique_ptrs
    std::array<std::shared_ptr<SwerveModule>, SwerveConstants::NUMBER_OF_MODULES> modules;

    // Gyro is uniquely owned by Odometry
    IGyroSensor* gyro;

    Pose pose;

    double lastDistances[SwerveConstants::NUMBER_OF_MODULES];
    double lastHeading;
};
