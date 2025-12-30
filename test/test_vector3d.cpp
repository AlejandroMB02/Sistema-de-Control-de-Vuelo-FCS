#include <gtest/gtest.h>
#include "math/Vector3D.hpp"

TEST(Vector3DTest, DefaultConstructor) {
    Vector3D v;
    EXPECT_DOUBLE_EQ(v.x, 0.0);
    EXPECT_DOUBLE_EQ(v.y, 0.0);
    EXPECT_DOUBLE_EQ(v.z, 0.0);
}

TEST(Vector3DTest, Norm) {
    Vector3D v(3.0, 4.0, 0.0);
    EXPECT_DOUBLE_EQ(v.norm(), 5.0);
}

TEST(Vector3DTest, Addition) {
    Vector3D a(1.0, 2.0, 3.0);
    Vector3D b(4.0, 5.0, 6.0);

    Vector3D c = a + b;

    EXPECT_DOUBLE_EQ(c.x, 5.0);
    EXPECT_DOUBLE_EQ(c.y, 7.0);
    EXPECT_DOUBLE_EQ(c.z, 9.0);
}
