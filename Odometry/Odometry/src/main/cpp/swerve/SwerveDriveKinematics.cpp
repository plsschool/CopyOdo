// #include "swerve/SwerveDriveKinematics.h"
// #include <cmath>
// #include "Math/MathUtils.h"
// #include "odometry/Pose.h"

// SwerveDriveKinematics::SwerveDriveKinematics(const std::array<Vector2D, NUM_WHEELS>& offsets)
//     : wheelOffsets(offsets) {}

// std::array<WheelModuleState, SwerveDriveKinematics::NUM_WHEELS> SwerveDriveKinematics::toWheelStates(const ChassisState& chassisState, const Pose& pose) {
    
//     std::array<WheelModuleState, NUM_WHEELS> states;

//     Vector2D velocity = chassisState.velocity;


//     if(fieldRelative){
//         velocity = velocity.rotate(-pose.getHeading());
//     }

//     for (int i = 0; i < NUM_WHEELS; ++i) {

//         Vector2D v_rot(-chassisState.omega * wheelOffsets[i].y, chassisState.omega * wheelOffsets[i].x);

//         Vector2D v_total = velocity + v_rot;

//         states[i].speed = v_total.magnitude();
//         states[i].angle = MathUtils::normalizeAngle(std::atan2(v_total.y, v_total.x));
//     }

//     normalizeWheelSpeeds(states, MAX_SPEED);
//     return states;
// }

// ChassisState SwerveDriveKinematics::toChassisState(const std::array<WheelModuleState, NUM_WHEELS>& wheelStates) 
// {
//     Vector2D avgVelocity(0,0);
//     double avgOmega = 0.0;

//     for (int i = 0; i < NUM_WHEELS; ++i) {
//         Vector2D v(std::cos(wheelStates[i].angle) * wheelStates[i].speed,
//                    std::sin(wheelStates[i].angle) * wheelStates[i].speed);

//         avgVelocity += v;

//         avgOmega += (v.x * wheelOffsets[i].y - v.y * wheelOffsets[i].x) /
//                     (wheelOffsets[i].x*wheelOffsets[i].x + wheelOffsets[i].y*wheelOffsets[i].y);
//     }

//     avgVelocity = avgVelocity * (1.0 / NUM_WHEELS);
//     avgOmega /= NUM_WHEELS;

//     return ChassisState(avgVelocity, avgOmega);
// }

// void SwerveDriveKinematics::normalizeWheelSpeeds(std::array<WheelModuleState, NUM_WHEELS>& states, double maxSpeed) {
//     // 1. Find the maximum speed among all wheel states
//     double higherSpeed = 0.0;
//     for (int i = 0; i < NUM_WHEELS; i++) {
//         // update higherSpeed if states[i].speed is bigger
//         if(states[i].speed > higherSpeed){
//             higherSpeed = states[i].speed;
//         }
//     }

//     // 2. Check if higherSpeed is greater than maxSpeed
//     if (higherSpeed > maxSpeed) {
//         // calculate scaleFactor once
//         double scaleFactor = maxSpeed/higherSpeed;

//         // 3. Apply scaleFactor to every wheel speed
//         for (int i = 0; i < NUM_WHEELS; i++) {
//             // scale states[i].speed
//             states[i].speed = states[i].speed * scaleFactor;
//         }
//     }
// }

// void SwerveDriveKinematics::toggleFieldRelativeControl(bool enabled){
//     fieldRelative = enabled;
// }

// bool SwerveDriveKinematics::isFieldRelative() const{
//     return fieldRelative;
// }
#include "swerve/SwerveDriveKinematics.h"
#include <cmath>
#include "math/MathUtils.h"
#include <stdio.h>

SwerveDriveKinematics::SwerveDriveKinematics(
    const std::array<Vector2D, NUM_WHEELS>& offsets
) : wheelOffsets(offsets) {}

std::array<WheelModuleState, SwerveDriveKinematics::NUM_WHEELS>
SwerveDriveKinematics::toWheelStates(const ChassisState& chassisState, const Pose& pose) {
    std::array<WheelModuleState, NUM_WHEELS> states;

    Vector2D velocity = chassisState.velocity;

    if (fieldRelative) {
        velocity = velocity.rotate(-pose.getHeading());
    }

    printf("Chassis: vel=(%.2f, %.2f) omega=%.2f rad/s\n",
       chassisState.velocity.x, chassisState.velocity.y, chassisState.omega);

    for (int i = 0; i < NUM_WHEELS; ++i) {
        Vector2D v_rot(
            -chassisState.omega * wheelOffsets[i].y,
             chassisState.omega * wheelOffsets[i].x
        );

        Vector2D v_total = velocity + v_rot;

        states[i].speed = v_total.magnitude();
        states[i].angle = MathUtils::normalizeAngle(std::atan2(v_total.y, v_total.x));
        printf("Wheel %d: v_rot=(%.2f, %.2f) v_total=(%.2f, %.2f)\n",
            i, v_rot.x, v_rot.y, v_total.x, v_total.y);
    }

    normalizeWheelSpeeds(states, MAX_SPEED);
    return states;
}

ChassisState SwerveDriveKinematics::toChassisState(
    const std::array<WheelModuleState, NUM_WHEELS>& wheelStates
) {
    Vector2D avgVelocity(0,0);
    double avgOmega = 0.0;

    for (int i = 0; i < NUM_WHEELS; ++i) {
        Vector2D v(
            std::cos(wheelStates[i].angle) * wheelStates[i].speed,
            std::sin(wheelStates[i].angle) * wheelStates[i].speed
        );

        avgVelocity += v;

        avgOmega += (v.x * wheelOffsets[i].y - v.y * wheelOffsets[i].x) /
                    (wheelOffsets[i].x*wheelOffsets[i].x + wheelOffsets[i].y*wheelOffsets[i].y);
    }

    avgVelocity = avgVelocity * (1.0 / NUM_WHEELS);
    avgOmega /= NUM_WHEELS;

    return ChassisState(avgVelocity, avgOmega);
}

void SwerveDriveKinematics::normalizeWheelSpeeds(
    std::array<WheelModuleState, NUM_WHEELS>& states, double maxSpeed
) {
    double higherSpeed = 0.0;
    for (int i = 0; i < NUM_WHEELS; i++) {
        if (states[i].speed > higherSpeed) {
            higherSpeed = states[i].speed;
        }
    }

    if (higherSpeed > maxSpeed) {
        double scaleFactor = maxSpeed / higherSpeed;
        for (int i = 0; i < NUM_WHEELS; i++) {
            states[i].speed *= scaleFactor;
        }
    }
}

void SwerveDriveKinematics::toggleFieldRelativeControl(bool enabled) {
    fieldRelative = enabled;
}

bool SwerveDriveKinematics::isFieldRelative() const {
    return fieldRelative;
}
