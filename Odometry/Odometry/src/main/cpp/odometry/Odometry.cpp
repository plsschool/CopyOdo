#include "odometry/Odometry.h"
#include "math/MathUtils.h"
#include <cmath>
#include <cstdio>

Odometry::Odometry(IEncoderSensor* enc, IGyroSensor* g)
    : encoder(enc), gyro(g), pose(), lastDistance(0.0), lastHeading(0.0) {}

void Odometry::update() {
    double currentDistance = encoder->getDistance();   // meters
    double currentHeading  = gyro->getHeading();      // radians

    double deltaDistance = currentDistance - lastDistance;
    double deltaHeading  = currentHeading - lastHeading;

    lastDistance = currentDistance;
    lastHeading  = currentHeading;

    pose.updatePose(deltaDistance, deltaHeading);

    printf("Pose: x=%.2f, y=%.2f, heading=%.2f deg\n",
           pose.position.x, pose.position.y,
           MathUtils::radToDeg(pose.getHeading()));
}

Pose Odometry::getPose() const {
    return pose;   // copy
}

void Odometry::resetPose(double x, double y, double heading) {
    pose.resetPose(x, y, heading);
    lastDistance = encoder->getDistance();
    lastHeading  = gyro->getHeading();
}
