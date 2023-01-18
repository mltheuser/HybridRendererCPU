#include "tensor.h"

#include <gtest/gtest.h>

TEST(TensorTest, tensor_creation) {
  Tensor t = Tensor({3, 2}, -1.5);
  std::array<int, 2> shape({3, 2});
  EXPECT_EQ(t.shape(), shape);
  std::vector<std::vector<float>> data(
      {{-1.5, -1.5}, {-1.5, -1.5}, {-1.5, -1.5}});
  EXPECT_EQ(t, data);

  t = Tensor({{1, 2, 3}, {4, 5, 6}});
  shape = std::array<int, 2>({2, 3});
  EXPECT_EQ(t.shape(), shape);
  data = std::vector<std::vector<float>>({{1, 2, 3}, {4, 5, 6}});
  EXPECT_EQ(t, data);

  t = Tensor(std::vector<std::vector<float>>({{1}}));
  shape = std::array<int, 2>({1, 1});
  EXPECT_EQ(t.shape(), shape);
  data = std::vector<std::vector<float>>({{1}});
  EXPECT_EQ(t, data);

  t = Tensor(std::vector<std::vector<float>>({}));
  shape = std::array<int, 2>({0, 0});
  EXPECT_EQ(t.shape(), shape);
  data = std::vector<std::vector<float>>({});
  EXPECT_EQ(t, data);

  t = Tensor(std::vector<std::vector<float>>({{}}));
  shape = std::array<int, 2>({1, 0});
  EXPECT_EQ(t.shape(), shape);
  data = std::vector<std::vector<float>>({{}});
  EXPECT_EQ(t, data);
};

TEST(TensorTest, addition) {
  Tensor t = Tensor({{1, 2}, {3, 4}});
  Tensor t2 = Tensor({{1, 2}, {3, 4}}) * -1;
  EXPECT_EQ(t + t2, Tensor({{0, 0}, {0, 0}}));
  EXPECT_EQ(t + 1, Tensor({{2, 3}, {4, 5}}));

  t2 = t2 * -1;
  EXPECT_EQ(t + t2, Tensor({{2, 4}, {6, 8}}));

  t2 = Tensor(std::vector<std::vector<float>>({{1}, {2}}));
  ASSERT_DEATH(t + t2, "Assertion");
};

TEST(TensorTest, multiplication) {
  Tensor t = Tensor({{1, 2, 3}, {4, 5, 6}});
  Tensor t2 = Tensor({{1, 2, 3}}).transpose();
  EXPECT_EQ(t * t2, Tensor(std::vector<std::vector<float>>({{14}, {32}})));
  EXPECT_EQ(t2 * 2, Tensor({{2, 4, 6}}).transpose());

  t2 = Tensor({{1, 2}}).transpose();
  ASSERT_DEATH(t * t2, "Assertion");
};

TEST(TensorTest, dot) {
  Tensor t = Tensor({{1, 2}, {3, 4}});
  Tensor t2 = Tensor({{1, 2}, {3, 4}});
  EXPECT_EQ(t.tensor_dot(t2), 1 * 1 + 2 * 2 + 3 * 3 + 4 * 4);

  t2 = Tensor(std::vector<std::vector<float>>({{1}, {2}}));
  ASSERT_DEATH(t.tensor_dot(t2), "Assertion");
};

TEST(TensorTest, sum) {
  Tensor t = Tensor({{1, 2}, {3, 4}});
  EXPECT_EQ(t.sum(), 1 + 2 + 3 + 4);
  t = Tensor({{1, 2, 3}, {-3, -2, -1}});
  EXPECT_EQ(t.sum(), 0);
  t = Tensor({});
  EXPECT_EQ(t.sum(), 0);
};

TEST(TensorTest, eq_operator) {
  Tensor t = Tensor({{1, 2}, {3, 4}});
  Tensor t2 = Tensor({{1, 2}, {3, 4}});
  EXPECT_TRUE(t == t2);
  std::vector<std::vector<float>> t2_data = {{1, 2}, {3, 4}};
  EXPECT_TRUE(t == t2_data);
};

TEST(TensorTest, transpose) {
  Tensor t({{1, 2, 3}, {4, 5, 6}});
  auto t_trans = t.transpose();
  EXPECT_EQ(t_trans, Tensor({{1, 4}, {2, 5}, {3, 6}}));
};