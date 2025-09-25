#pragma once
#include <cmath>

// Vector2D class: represents a 2D vector with basic operations
class Vector2D {
public:
    double x;   // X component
    double y;   // Y component

    // Constructor: initializes vector with optional x and y (defaults = 0)
    Vector2D(double x_=0, double y_=0);

    // Returns the magnitude (length) of the vector
    double magnitude() const;

    // Normalizes the vector (makes its length = 1)
    void normalize();

    // Operator overload: vector addition
    Vector2D operator+(const Vector2D& other) const;

    // Operator overload: scalar multiplication
    Vector2D operator*(double scalar) const;

    Vector2D& operator+=(const Vector2D& other);

    Vector2D rotate(double angle) const;
};
