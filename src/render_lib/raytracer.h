#pragma once

#include <memory>

#include "helpers/helpers.h"
#include "mesh.h"

struct Ray {
  Vector<3>& origin;
  Vector<3>& direction;
};

struct RayHit {};

bool intersects_plane(Ray& ray, Vector<3>& plane_origin,
                      Vector<3>& plane_normal, float& intersection_distance);

float edge_function_3D(const Vector<3>& origin, const Vector<3>& target_1,
                       const Vector<3>& target_2,
                       const Vector<3>& plane_normal);

bool is_inside_triangle_3D(Vector<3>& intersection_point,
                           std::array<Vector<3>, 3>& world_space_triangle,
                           Vector<3>& plane_normal);

namespace Raytracer {
// As its currently only used for shadow rays, always returns first hit.
std::unique_ptr<RayHit> cast_ray(
    Ray& ray, std::vector<std::array<Vector<3>, 3>>& world_space_triangles,
    std::vector<Vector<3>>& face_normals, std::vector<Mesh*>& meshes);
};  // namespace Raytracer