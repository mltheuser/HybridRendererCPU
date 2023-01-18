#include "helpers.h"

#include <math.h>

float degrees_to_radians(float degrees) { return degrees * M_PI / 180; }

float radians_to_degrees(float radians) { return radians * 180 / M_PI; }

Vector<4> homogeneous_transform(Vector<3>& v, const Tensor& mat) {
  Vector<4> tmp({v[0], v[1], v[2], 1});
  Vector<4> output = tmp * mat;
  return output;
}

Vector<3> homogeneous_transform_affine(Vector<3>& v, Tensor& mat) {
  return Vector<3>({
    v[0] * mat[0][0] + v[1] * mat[1][0] + v[2] * mat[2][0] + mat[3][0], 
    v[0] * mat[0][1] + v[1] * mat[1][1] + v[2] * mat[2][1] + mat[3][1], 
    v[0] * mat[0][2] + v[1] * mat[1][2] + v[2] * mat[2][2] + mat[3][2], 
  });
}

Vector<3> cross(Vector<3>& a, Vector<3>& b) {
  return Vector<3>({a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]});
}

Tensor invert_transpose(Tensor& A) { 
  assert(A.shape()[0] >= 3 && A.shape()[1] >= 3);
  Tensor A_inv_trans({3, 3}, 0);
  double determinant =  +A(0,0)*(A(1,1)*A(2,2)-A(2,1)*A(1,2))
                        -A(0,1)*(A(1,0)*A(2,2)-A(1,2)*A(2,0))
                        +A(0,2)*(A(1,0)*A(2,1)-A(1,1)*A(2,0));
  double invdet = 1/determinant;
  A_inv_trans(0,0) =  (A(1,1)*A(2,2)-A(2,1)*A(1,2))*invdet;
  A_inv_trans(1,0) = -(A(0,1)*A(2,2)-A(0,2)*A(2,1))*invdet;
  A_inv_trans(2,0) =  (A(0,1)*A(1,2)-A(0,2)*A(1,1))*invdet;
  A_inv_trans(0,1) = -(A(1,0)*A(2,2)-A(1,2)*A(2,0))*invdet;
  A_inv_trans(1,1) =  (A(0,0)*A(2,2)-A(0,2)*A(2,0))*invdet;
  A_inv_trans(2,1) = -(A(0,0)*A(1,2)-A(1,0)*A(0,2))*invdet;
  A_inv_trans(0,2) =  (A(1,0)*A(2,1)-A(2,0)*A(1,1))*invdet;
  A_inv_trans(1,2) = -(A(0,0)*A(2,1)-A(2,0)*A(0,1))*invdet;
  A_inv_trans(2,2) =  (A(0,0)*A(1,1)-A(1,0)*A(0,1))*invdet;

  return A_inv_trans;
};
