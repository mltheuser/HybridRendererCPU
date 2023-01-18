#pragma once

#include "boxshape.h"

template <typename B, typename T>
class Buffer {
 public:
  Buffer(BoxShape<B> shape, T constant)
      : shape_{shape}, data_{std::vector<T>(shape.size(), constant)}, constant_{constant} {}
  T& operator[](const int& index) { return data_[index]; }
  void reset() { std::fill(data_.begin(), data_.end(), constant_); }

  operator const void*() const { return &data_[0]; }

  const B& width() { return shape_.width; }
  const B& height() { return shape_.height; }
  const B& depth() { return shape_.depth; }
  const BoxShape<B>& shape() { return shape_; }

 private:
  T constant_;
  BoxShape<B> shape_;
  std::vector<T> data_;
};