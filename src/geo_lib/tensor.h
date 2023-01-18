# pragma once

#include <vector>
#include <array>

class Tensor {
 public:
  static Tensor identity(int size);
  static Tensor clamp(Tensor& tensor, float min_val, float max_val);

  Tensor(std::array<int, 2> shape, float constant);
  Tensor(std::vector<std::vector<float>>&& data);

  const std::array<int, 2>& shape() const;
  const std::vector<std::vector<float>>& data() const;

  void print() const;

  float norm() const;

  float sum() const;

  float& operator()(int row, int col);
  std::vector<float>& operator[](int index);

  bool operator==(const Tensor& otherTensor) const;
  bool operator==(const std::vector<std::vector<float>>& data) const;

  Tensor operator+(const Tensor& otherTensor) const;
  Tensor operator+(const float scalar) const;

  Tensor operator-(const Tensor& otherTensor) const;
  Tensor operator-(const float scalar) const;

  Tensor operator*(const Tensor& otherTensor) const;
  Tensor operator*(const float factor) const;

  Tensor operator/(const float factor) const;

  float tensor_dot(const Tensor& otherTensor) const;

  Tensor transpose() const;
 protected:
  size_t size() const;
  static std::array<int, 2> infer_shape(const std::vector<std::vector<float>>& data);

  std::array<int, 2> shape_;
  std::vector<std::vector<float>> data_;
};