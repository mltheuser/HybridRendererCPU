#pragma once

#include "vector.h"

float degrees_to_radians(float degrees);
float radians_to_degrees(float radians);

Vector<4> homogeneous_transform(Vector<3>& v, const Tensor& mat);
Vector<3> homogeneous_transform_affine(Vector<3>& v, Tensor& mat);

Vector<3> cross(Vector<3>& a, Vector<3>& b);

Tensor invert_transpose(Tensor& A);