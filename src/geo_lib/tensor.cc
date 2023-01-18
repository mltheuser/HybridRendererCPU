#include "tensor.h"

#include <math.h>

#include <iostream>
#include <numeric>
#include <algorithm>
#include <cassert>

Tensor Tensor::identity(int size) { 
  Tensor output({size, size}, 0);
  for (int i=0; i<size; ++i) {
    output[i][i] = 1;
  }
  return output;
}

Tensor Tensor::clamp(Tensor& tensor, float min_val, float max_val) {
  int rows = tensor.shape_[0];
  int cols = tensor.shape_[1];

  Tensor output = Tensor(tensor);
  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      auto& val = tensor.data_[row][col];
      if (!(min_val <= val <= max_val)) {
        output.data_[row][col] = std::clamp(val, min_val, max_val);
      }
    }
  }
  return output;
}

Tensor::Tensor(std::array<int, 2> shape, float constant) : shape_{shape} {
  data_ = std::vector<std::vector<float>>(
      shape_[0], std::vector<float>(shape_[1], constant));
}

Tensor::Tensor(std::vector<std::vector<float>>&& data) : data_{data} {
  shape_ = infer_shape(data);
}

const std::array<int, 2>& Tensor::shape() const { return shape_; }

const std::vector<std::vector<float>>& Tensor::data() const { return data_; }

std::array<int, 2> Tensor::infer_shape(
    const std::vector<std::vector<float>>& data) {
  int rows = data.size();
  if (rows == 0) {
    return {0, 0};
  }

  int cols = data[0].size();
  for (int row = 1; row < rows; row++) {
    if (cols != data[row].size()) {
      std::invalid_argument("invalid matrix shape. (different number of cols per row)");
    }
  }
  return {rows, cols};
}

size_t Tensor::size() const {
  int output = 1;
  for (const auto& dim : shape_) {
    output *= dim;
  }
  return (size_t)output;
}

void Tensor::print() const {
  int rows = shape_[0];
  int cols = shape_[1];

  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      std::cout << data_[row][col] << " ";
    }
    std::cout << std::endl;
  }
}

Tensor Tensor::operator+(const Tensor& otherTensor) const {
  assert(shape_ == otherTensor.shape_);

  int rows = shape_[0];
  int cols = shape_[1];

  Tensor output = Tensor(*this);
  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      output.data_[row][col] += otherTensor.data_[row][col];
    }
  }
  return output;
}

Tensor Tensor::operator+(const float scalar) const {
  int rows = shape_[0];
  int cols = shape_[1];

  Tensor output = Tensor(*this);
  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      output[row][col] += scalar;
    }
  }
  return output;
}

Tensor Tensor::operator-(const Tensor& otherTensor) const {
  return *this + otherTensor * -1;
}

Tensor Tensor::operator-(const float scalar) const {
  return *this + scalar * -1;
}

Tensor Tensor::operator*(const Tensor& otherTensor) const {
  int rows = shape_[0];
  int cols = shape_[1];

  int otherRows = otherTensor.shape_[0];
  int otherCols = otherTensor.shape_[1];

  assert(cols == otherRows);

  Tensor output = Tensor(std::array<int, 2>{rows, otherCols}, 0);
  for (int row = 0; row < rows; row++) {
    for (int col2 = 0; col2 < otherCols; col2++) {
      for (int col = 0; col < cols; col++) {
        output.data_[row][col2] +=
            data_[row][col] * otherTensor.data_[col][col2];
      }
    }
  }
  return output;
}

Tensor Tensor::operator*(const float factor) const {
  int rows = shape_[0];
  int cols = shape_[1];

  Tensor output = Tensor(*this);
  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      output.data_[row][col] *= factor;
    }
  }
  return output;
}

Tensor Tensor::operator/(const float factor) const {
  return *this * (1/factor);
}

float Tensor::tensor_dot(const Tensor& otherTensor) const {
  assert(shape_ == otherTensor.shape_);

  int rows = shape_[0];
  int cols = shape_[1];

  float output = 0;
  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      output += data_[row][col] * otherTensor.data_[row][col];
    }
  }
  return output;
}

Tensor Tensor::transpose() const {
  int rows = shape_[0];
  int cols = shape_[1];

  Tensor output = Tensor(std::array<int, 2>{cols, rows}, 0);
  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      output.data_[col][row] = this->data_[row][col];
    }
  }
  return output;
}

float Tensor::norm() const {
  return sqrt(this->tensor_dot(*this));
}

std::vector<float>& Tensor::operator[](int index) { return data_[index]; }

float Tensor::sum() const {
  int rows = shape_[0];
  int cols = shape_[1];

  float output = 0;
  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      output += data_[row][col];
    }
  }
  return output;
}

float& Tensor::operator()(int row, int col) {
  return data_[row][col];
}

bool Tensor::operator==(const Tensor& otherTensor) const {
  return data_ == otherTensor.data_;
}

bool Tensor::operator==(const std::vector<std::vector<float>>& data) const {
  return data_ == data;
}
