#pragma once

#include "inhabitant.h"
#include "helpers/boxshape.h"

class Camera : public Inhabitant {
 public:
  void set_image_shape(const BoxShape<int> image_shape);
  Tensor projection_matrix();

  const BoxShape<int>& image_shape() const;
  const BoxShape<float>& screen_shape() const;

  float clip_near_ = 1;
  float clip_far_ = 100;

 private:
  float field_of_view_in_degree_ = 60;
  float aspect_ratio_;

  BoxShape<float> screen_shape_ = BoxShape<float>(0, 0, 0);
  BoxShape<int> image_shape_ = BoxShape<int>(0, 0, 0);

  void update_aspect_ration();
  void update_screen_shape();
};