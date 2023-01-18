#include "inhabitant.h"

#include "../geo_lib/helpers.h"

#include <math.h>

void Inhabitant::translate(const std::array<float, 3> translation) {
  local_to_world_mat_[3][0] += translation[0];
  local_to_world_mat_[3][1] += translation[1];
  local_to_world_mat_[3][2] += translation[2];
}

void Inhabitant::scale(std::array<float, 3> scaling) {
  Tensor scale_mat = Tensor::identity(4);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      local_to_world_mat_[j][i] *= scaling[i];
    }
  }
  local_to_world_mat_ = local_to_world_mat_ * scale_mat;
}

void Inhabitant::rotate(float degree_around_x, float degree_around_y,
                        float degree_around_z) {
  Tensor rot_matrix = Tensor::identity(4);
  if (degree_around_x != 0) {
    const float radians_around_x = degrees_to_radians(degree_around_x);
    const Tensor rot_matrix_axis = Tensor({
        {1, 0, 0, 0},
        {0, cos(radians_around_x), sin(radians_around_x), 0},
        {0, -sin(radians_around_x), cos(radians_around_x), 0},
        {0, 0, 0, 1},
    });
    rot_matrix = rot_matrix * rot_matrix_axis;
  }

  if (degree_around_y != 0) {
    const float radians_around_y = degrees_to_radians(degree_around_y);
    const Tensor rot_matrix_axis = Tensor({
        {cos(radians_around_y), 0, -sin(radians_around_y), 0},
        {0, 1, 0, 0},
        {sin(radians_around_y), 0, cos(radians_around_y), 0},
        {0, 0, 0, 1},
    });
    rot_matrix = rot_matrix * rot_matrix_axis;
  }

  if (degree_around_z != 0) {
    const float radians_around_z = degrees_to_radians(degree_around_z);
    const Tensor rot_matrix_axis = Tensor({
        {cos(radians_around_z), sin(radians_around_z), 0, 0},
        {-sin(radians_around_z), cos(radians_around_z), 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1},
    });
    rot_matrix = rot_matrix * rot_matrix_axis;
  }
  Vector<3> translation = Vector<3>({local_to_world_mat_[3][0], local_to_world_mat_[3][1], local_to_world_mat_[3][2]});
  Vector<3> inverse_translation = translation * -1;
  this->translate(inverse_translation);
  local_to_world_mat_ = local_to_world_mat_ * rot_matrix;
  this->translate(translation);
}