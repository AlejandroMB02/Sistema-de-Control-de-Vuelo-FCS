#include "math/Vector3D.hpp"
#include <cmath>

Vector3D::Vector3D() : x(0.0), y(0.0), z(0.0) {}

Vector3D::Vector3D(double x, double y, double z)
    : x(x), y(y), z(z) {}

double Vector3D::norm() const {
    return std::sqrt(x * x + y * y + z * z);
}

Vector3D Vector3D::operator+(const Vector3D& other) const {
    return Vector3D(
        x + other.x,
        y + other.y,
        z + other.z
    );
}