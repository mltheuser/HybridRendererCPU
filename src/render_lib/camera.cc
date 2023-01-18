#include "camera.h"

#include <math.h>

#include "../geo_lib/helpers.h"

void Camera::set_image_shape(const BoxShape<int> image_shape) {
  image_shape_ = image_shape;
  update_aspect_ration();
  update_screen_shape();
}

Tensor Camera::projection_matrix() {
  return Tensor({
      {4 * clip_near_ / screen_shape_.width, 0, 0, 0},
      {0, 4 * clip_near_ / screen_shape_.height, 0, 0},
      {-0.5, -0.5, -1 / (clip_far_ - clip_near_), -1},
      {0, 0, -clip_near_ / (clip_far_ - clip_near_), 0},
  });
}

const BoxShape<int>& Camera::image_shape() const { return image_shape_; }

const BoxShape<float>& Camera::screen_shape() const { return screen_shape_; }

void Camera::update_aspect_ration() {
  aspect_ratio_ = image_shape_.width / image_shape_.height;
}

void Camera::update_screen_shape() {
  screen_shape_.width =
      2 * tan(degrees_to_radians(field_of_view_in_degree_ / 2)) * clip_near_;
  screen_shape_.height = screen_shape_.width / aspect_ratio_;
}