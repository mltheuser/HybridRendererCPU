#pragma once

#include "../inhabitant.h"
#include "raytracer.h"

class Light : public Inhabitant {
 public:
  Vector<3> virtual shade(
      Vector<3>& albedo, Vector<3>& point_world_space,
      Vector<3>& view_direction, Vector<3>& point_normal_world_space,
      std::vector<Mesh*>& meshes,
      std::vector<std::array<Vector<3>, 3>>& world_space_triangles,
      std::vector<Vector<3>>& face_normals) = 0;

 protected:
  bool is_in_shadow(
      Vector<3>& point_normal_world_space,
      Vector<3>& normalized_light_direction, std::vector<Mesh*> meshes,
      std::vector<std::array<Vector<3>, 3>>& world_space_triangles,
      std::vector<Vector<3>>& face_normals) {
    Ray ray = {point_normal_world_space, normalized_light_direction};
    return (Raytracer::cast_ray(ray, world_space_triangles, face_normals,
                                meshes) != nullptr);
  }
};