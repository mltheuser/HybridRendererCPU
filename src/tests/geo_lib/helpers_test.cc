#include "helpers.h"

#include <gtest/gtest.h>
#include <math.h>

TEST(GeoHelpers, degrees_to_radians_test) {
  EXPECT_FLOAT_EQ(degrees_to_radians(360), 360 * M_PI / 180);
  EXPECT_FLOAT_EQ(degrees_to_radians(45), 45 * M_PI / 180);
}

TEST(GeoHelpers, radians_to_degrees_test) {
  EXPECT_FLOAT_EQ(radians_to_degrees(360 * M_PI / 180), 360);
  EXPECT_FLOAT_EQ(radians_to_degrees(45 * M_PI / 180), 45);
}

TEST(GeoHelpers, cross_test) {
  Vector<3> v = Vector<3>({1, 2, 3});
  Vector<3> v2 = Vector<3>({4, 5, 6});
  Vector<3> res = Vector<3>({-3, 6, -3});
  EXPECT_EQ(cross(v, v2), res);
}

TEST(GeoHelpers, invert_transpose_test) {
  Tensor t = Tensor({{1, 0}, {-1, 2}});
  EXPECT_DEATH(invert_transpose(t), "Assertion");
  t = Tensor({{1, 2, -1}, {2, 1, 2}, {-1, 2, 1}});
  Tensor transposed_inverse = Tensor(
      {{3 / 16, 1 / 4, -5 / 16}, {1 / 4, 0, 1 / 4}, {-5 / 16, 1 / 4, 3 / 16}});
  EXPECT_EQ(t * invert_transpose(t), Tensor::identity(3));
}