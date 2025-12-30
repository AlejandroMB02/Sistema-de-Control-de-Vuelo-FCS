#pragma once

class Vector3D {
public:
    double x;
    double y;
    double z;

    Vector3D();
    Vector3D(double x, double y, double z);

    [[nodiscard]] double norm() const;
    Vector3D operator+(const Vector3D& other) const;
};
