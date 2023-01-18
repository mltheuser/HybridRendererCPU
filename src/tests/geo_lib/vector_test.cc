#include "vector.h"

#include <gtest/gtest.h>

TEST(VectorTest, creation) {
  Vector<2> v = Vector<2>(0);
  EXPECT_EQ(Tensor({{0, 0}}), v);
  v = Vector<2>(1);
  EXPECT_EQ(Tensor({{1, 1}}), v);
  v = Vector<2>({1, 1});
  EXPECT_EQ(Tensor({{1, 1}}), v);
  ASSERT_DEATH(v = Vector<2>({1, 1, 1}), "Assertion");
  Tensor t = Tensor({{1, 1, 1}});
  Vector<3> v2 = t;
  EXPECT_EQ(t, v2);
  ASSERT_DEATH(v = t, "Assertion");
}

TEST(VectorTest, mul_point_wise_test) {
  Vector<2> v = Vector<2>({1, 2});
  Vector<2> v2 = Vector<2>({3, 4});
  EXPECT_EQ(v.mul_point_wise(v2), Vector<2>({3, 8}));
}

TEST(VectorTest, to_array) {
  Vector<2> v = Vector<2>({1, 2});
  std::array<float, 2> v_as_array = v;
  std::array<float, 2> comp_array = {1, 2};
  EXPECT_EQ(comp_array, v_as_array);
}