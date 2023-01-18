#include "helpers.h"

#include "../../geo_lib/helpers.h"


std::array<Vector<3>, 3> transform_triangle(Vector<3>* triangle,
                                            Tensor& matrix) {
  std::array<Vector<3>, 3> output;
  for (int i = 0; i < 3; ++i) {
    output[i] = homogeneous_transform_affine(triangle[i], matrix);
  }
  return output;
}

Vector<3> get_plane_normal(std::array<Vector<3>, 3> triangle) {
    Vector<3> a = triangle[1] - triangle[0];
    Vector<3> b = triangle[2] - triangle[0];
    return cross(a, b).normalize();
}