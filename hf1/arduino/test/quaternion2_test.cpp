#include <gtest/gtest.h>
#include "quaternion2.h"

TEST(Quaternion2Test, EmptyConstructedReturnsAngleZero) {
  EXPECT_NEAR(Quaternion2f().angle(), 0, /*abs_error=*/1e-6);
}

TEST(Quaternion2Test, AngleConstructedReturnsAngle) {
  EXPECT_NEAR(Quaternion2f(M_PI / 8).angle(), M_PI / 8, /*abs_error=*/1e-6);
}

TEST(Quaternion2Test, SinAReturnsSineOfAngle) {
  EXPECT_NEAR(Quaternion2f(M_PI / 8).sin_angle(), sinf(M_PI / 8), /*abs_error=*/1e-6);
}

TEST(Quaternion2Test, CosAReturnsCosineOfAngle) {
  EXPECT_NEAR(Quaternion2f(M_PI / 8).cos_angle(), cosf(M_PI / 8), /*abs_error=*/1e-6);
}

TEST(Quaternion2Test, TanAReturnsTangentOfAngle) {
  EXPECT_NEAR(Quaternion2f(M_PI / 8).tan_angle(), tanf(M_PI / 8), /*abs_error=*/1e-6);
}

TEST(Quaternion2Test, SumReturnsAngleOfInternalVectorSum) {
  EXPECT_NEAR((Quaternion2f(M_PI / 2) + Quaternion2f(0)).angle(), M_PI / 4, /*abs_error=*/1e-6);
  EXPECT_NEAR((Quaternion2f(M_PI) + Quaternion2f(-M_PI)).angle(), M_PI, /*abs_error=*/1e-6);
  EXPECT_NEAR((Quaternion2f(M_PI + 0.1) + Quaternion2f(-M_PI - 0.1)).angle(), M_PI, /*abs_error=*/1e-6);
}

TEST(Quaternion2Test, FactorMultipliesSineByFactor) {
  EXPECT_NEAR((Quaternion2f(M_PI / 2) * 10).sin_angle(), sinf(M_PI / 2) * 10, /*abs_error=*/1e-6);
}

TEST(Quaternion2Test, FactorMultipliesCosineByFactor) {
  EXPECT_NEAR((Quaternion2f(M_PI / 2) * 10).cos_angle(), cosf(M_PI / 2) * 10, /*abs_error=*/1e-6);
}

TEST(Quaternion2Test, FactorIsCommutative) {
  EXPECT_NEAR((Quaternion2f(M_PI / 2) * 10).cos_angle(), (10.0f * Quaternion2f(M_PI / 2)).cos_angle(), /*abs_error=*/1e-6);
}

TEST(Quaternion2Test, FactorResultsInSameTangent) {
  EXPECT_NEAR((Quaternion2f(M_PI / 2) * 10).tan_angle(), tanf(M_PI / 2), /*abs_error=*/1e-6);
}

TEST(Quaternion2Test, FactorResultsInSameAngle) {
  EXPECT_NEAR((Quaternion2f(M_PI / 2) * 10).angle(), M_PI / 2, /*abs_error=*/1e-6);
}

TEST(Quaternion2Test, NormReturnsCorrectNorm) {
  EXPECT_NEAR((Quaternion2f(M_PI / 4)).norm(), 1, /*abs_error=*/1e-6);
  EXPECT_NEAR((Quaternion2f(M_PI / 4) * 10).norm(), 10, /*abs_error=*/1e-6);
}

// TODO: test interpolation functions.
