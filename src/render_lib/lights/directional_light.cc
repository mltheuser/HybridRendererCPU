#include "directional_light.h"

#include "math.h"

DirectionalLight::DirectionalLight(Vector<3> direction, Vector<3> color,
                                   float intensity, bool cast_shadow)
    : color_{color}, intensity_{intensity}, cast_shadow_{cast_shadow} {
  this->set_direction(direction);
}

Vector<3> DirectionalLight::shade(
    Vector<3>& albedo, Vector<3>& point_world_space, Vector<3>& view_direction,
    Vector<3>& point_normal_world_space, std::vector<Mesh*>& meshes,
    std::vector<std::array<Vector<3>, 3>>& world_space_triangles,
    std::vector<Vector<3>>& face_normals) {
  Vector<3> normed_light_direction = this->direction();
  float light_exposure = std::max(
      0.f, point_normal_world_space.tensor_dot(normed_light_direction));

  Vector<3> hitColor = albedo.mul_point_wise(this->color_) * light_exposure /
                       M_PI * this->intensity_;

  float shadow_bias = 1e-4;
  Vector<3> shadow_ray_origin =
      point_world_space + point_normal_world_space * shadow_bias;

  if (hitColor.sum() > 0 &&
      (!cast_shadow_ ||
       !is_in_shadow(shadow_ray_origin, normed_light_direction, meshes,
                     world_space_triangles, face_normals))) {
    return hitColor;
  }
  return Vector<3>(0);
}

void DirectionalLight::set_direction(Vector<3> direction) {
  direction = direction.normalize();
  local_to_world_mat_(2, 0) = direction[0];
  local_to_world_mat_(2, 1) = direction[1];
  local_to_world_mat_(2, 2) = direction[2];
}

Vector<3> DirectionalLight::direction() {
  return Vector<3>({local_to_world_mat_(2, 0), local_to_world_mat_(2, 1),
                    local_to_world_mat_(2, 2)});
}