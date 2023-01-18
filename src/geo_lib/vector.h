#pragma once

#include "tensor.h"

#include <cassert>

template <size_t S>
class Vector : public Tensor {
 public:
  Vector(float constant = 0) : Tensor({1, S}, constant){};
  Vector(std::vector<float>&& data) : Tensor({data}) { assert_shape(shape_); }
  Vector(Tensor t) : Tensor({1, S}, 0) {
    assert_shape(t.shape());
    std::copy(t.data().begin(), t.data().end(), this->data_.begin());
  };

  Vector<S> normalize() { return *this / this->norm(); }

  Vector<S> mul_point_wise(Vector<S>& otherVector) {
    Vector<S> output(0);
    for (int i = 0; i < S; i++) {
      output[i] = (*this)[i] * otherVector[i];
    }
    return output;
  }

  operator std::array<float, S>() const {
    auto output = std::array<float, S>();
    std::copy(data_[0].begin(), data_[0].end(), output.begin());
    return output;
  }

  void assert_shape(const std::array<int, 2>& shape) const {
    assert(shape[0] == 1 && shape[1] == S);
  }

  float& operator[](int index) { return data_[0][index]; };
};