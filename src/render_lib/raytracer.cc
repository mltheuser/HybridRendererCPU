#include "raytracer.h"

#include "../geo_lib/helpers.h"

bool intersects_plane(Ray& ray, Vector<3>& plane_origin,
                      Vector<3>& plane_normal, float& intersection_distance) {
  float denom = plane_normal.tensor_dot(ray.direction);
  if (denom < 1e-6) {
    intersection_distance =
        (plane_origin - ray.origin).tensor_dot(plane_normal) / denom;
    return (intersection_distance >= 0);
  }
  return false;
}

float edge_function_3D(const Vector<3>& origin, const Vector<3>& target_1,
                       const Vector<3>& target_2,
                       const Vector<3>& plane_normal) {
  Vector<3> v1 = target_1 - origin;
  Vector<3> v2 = target_2 - origin;

  return plane_normal.tensor_dot(cross(v1, v2));
}

bool is_inside_triangle_3D(Vector<3>& intersection_point,
                           std::array<Vector<3>, 3>& world_space_triangle,
                           Vector<3>& plane_normal) {
  if (edge_function_3D(world_space_triangle[0], world_space_triangle[1],
                       intersection_point, plane_normal) > 0 &&
      edge_function_3D(world_space_triangle[1], world_space_triangle[2],
                       intersection_point, plane_normal) > 0 &&
      edge_function_3D(world_space_triangle[2], world_space_triangle[0],
                       intersection_point, plane_normal) > 0) {
    return true;
  }
  return false;
}

std::unique_ptr<RayHit> Raytracer::cast_ray(
    Ray& ray, std::vector<std::array<Vector<3>, 3>>& world_space_triangles,
    std::vector<Vector<3>>& face_normals, std::vector<Mesh*>& meshes) {
  int triangle_index_offset = 0;
  for (auto& mesh_ptr : meshes) {
    Mesh& mesh = *mesh_ptr;
    int num_triangles = mesh.num_triangles();
    for (int triangle_index = 0; triangle_index < num_triangles;
         triangle_index++) {
      int global_triangle_index = triangle_index_offset + triangle_index;
      std::array<Vector<3>, 3>& world_space_triangle = world_space_triangles[global_triangle_index];
      Vector<3>& plane_normal = face_normals[global_triangle_index];
      Vector<3>& plane_origin = world_space_triangle[0];

      // plane intersection test
      float intersection_distance;
      if (intersects_plane(ray, plane_origin, plane_normal,
                           intersection_distance)) {
        // compute intersection point
        Vector<3> intersection_point =
            ray.origin + ray.direction * intersection_distance;
        if (is_inside_triangle_3D(intersection_point, world_space_triangle,
                                  plane_normal)) {
          return std::make_unique<RayHit>();
        }
      }
    }
    triangle_index_offset += num_triangles;
  }
  return nullptr;
}
