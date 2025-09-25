#pragma once
#include "odometry/Pose.h"
#include "hardware/IEncoderSensor.h"
#include "hardware/IGyroSensor.h"

class Odometry {
private:
    IEncoderSensor* encoder;
    IGyroSensor* gyro;
    Pose pose;
    double lastDistance;
    double lastHeading;

public:
    Odometry(IEncoderSensor* enc, IGyroSensor* g);

    void update();               // Call every cycle to update pose
    Pose getPose() const;        // Returns a copy of current pose
    void resetPose(double x = 0, double y = 0, double heading = 0);
};
