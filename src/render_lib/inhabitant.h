#pragma once

#include "../geo_lib/tensor.h"
#include "../geo_lib/vector.h"

class Inhabitant {
 public:
  void translate(std::array<float, 3> translation);
  void scale(std::array<float, 3> scaling);
  void rotate(float around_x, float around_y, float around_z);

  Tensor local_to_world_mat_ = Tensor::identity(4);
};