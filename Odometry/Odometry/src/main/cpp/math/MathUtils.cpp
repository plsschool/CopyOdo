// make unit tests for this
#define _USE_MATH_DEFINES
#include "math/MathUtils.h"
#include <cmath>

double MathUtils::radToDeg(double radians){
    return radians * 180.0 / PI;
}

double MathUtils::degToRad(double degrees){
    return degrees * PI / 180.0;
}

double MathUtils::normalizeAngle(double radians){
    while (radians > PI) radians -= 2 * PI;
    while (radians <= -PI) radians += 2 * PI;
    return radians;
}