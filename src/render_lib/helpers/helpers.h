#pragma once

#include "../../geo_lib/vector.h"

std::array<Vector<3>, 3> transform_triangle(Vector<3>* triangle,
                                            Tensor& matrix);

Vector<3> get_plane_normal(std::array<Vector<3>, 3> triangle);