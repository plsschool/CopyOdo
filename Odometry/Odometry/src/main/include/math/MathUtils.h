#pragma once

class MathUtils {
    public:
    static constexpr double PI = 3.14159265358979323846;
    // Converts radians to degrees
    static double radToDeg(double radians);

    // Converts degrees to radians
    static double degToRad(double degrees);

    // Normalizes angle to [-pi, pi]/[-180, +180]
    static double normalizeAngle(double radians);
};
