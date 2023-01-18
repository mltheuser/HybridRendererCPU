#include "rasterizer.h"

#include <math.h>

#include <algorithm>

#include "../geo_lib/helpers.h"

std::array<Vector<4>, 3> project_vertices(std::array<Vector<3>, 3>& triangle,
                                          Tensor& model_to_screen_matrix) {
  std::array<Vector<4>, 3> output;
  for (int i = 0; i < 3; ++i) {
    auto vertex = triangle[i];
    output[i] = homogeneous_transform(vertex, model_to_screen_matrix);
  }
  return output;
}

std::vector<std::array<Vector<4>, 3>> clip_vertices(
    std::array<Vector<4>, 3>& triangle_clipping_space) {
  std::vector<std::array<Vector<4>, 3>> output;

  // PLACEHOLDER
  output.push_back(triangle_clipping_space);

  return output;
}

std::vector<std::array<Vector<3>, 3>> transform_to_raster_space(
    std::vector<std::array<Vector<4>, 3>>& clipped_triangles,
    const BoxShape<int>& image_shape) {
  std::vector<std::array<Vector<3>, 3>> output;
  output.reserve(clipped_triangles.size());

  for (int i = 0; i < clipped_triangles.size(); ++i) {
    std::array<Vector<4>, 3>& clipping_space_triangle = clipped_triangles[i];

    output.push_back(std::array<Vector<3>, 3>());
    std::array<Vector<3>, 3>& raster_space_triangle = output[i];

    for (int vertex_id = 0; vertex_id < 3; ++vertex_id) {
      Vector<4>& clipping_space_vertex = clipping_space_triangle[vertex_id];
      raster_space_triangle[vertex_id] =
          Vector<3>({clipping_space_vertex[0] / clipping_space_vertex[3] *
                         (image_shape.width + 1),
                     clipping_space_vertex[1] / clipping_space_vertex[3] *
                         (image_shape.height + 1),
                     clipping_space_vertex[2]});
    }
  }
  return output;
}

std::array<Vector<3>, 3> normals_to_world(Vector<3>* local_normals,
                                          Tensor& normal_to_world_mat) {
  std::array<Vector<3>, 3> world_normals;
  for (int j = 0; j < 3; ++j) {
    world_normals[j] = local_normals[j] * normal_to_world_mat;
  }
  return world_normals;
}

bool is_backface(std::array<Vector<3>, 3>& world_verticies,
                 std::array<Vector<3>, 3>& world_normals, Camera& camera) {
  auto& camera_mat = camera.local_to_world_mat_;
  auto camera_position =
      Vector<3>({camera_mat(3, 0), camera_mat(3, 1), camera_mat(3, 2)});

  for (int vertex_id = 0; vertex_id < 3; ++vertex_id) {
    auto camera_view_direction = world_verticies[vertex_id] - camera_position;
    if (world_normals[vertex_id].tensor_dot(camera_view_direction) < 0) {
      return false;
    }
  }
  return true;
}

float edge_function_2D(Tensor& origin, Tensor& target_1, Tensor& target_2) {
  auto v1 =
      Vector<2>({target_1(0, 0) - origin(0, 0), target_1(0, 1) - origin(0, 1)});
  auto v2 =
      Vector<2>({target_2(0, 0) - origin(0, 0), target_2(0, 1) - origin(0, 1)});

  return (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

bool is_inside_triangle_2D(Vector<2>& point,
                           std::array<Vector<3>, 3>& raster_space_triangle,
                           float triangle_area,
                           std::array<float, 3>& barycentric_coords) {
  barycentric_coords[0] = edge_function_2D(raster_space_triangle[1],
                                           raster_space_triangle[2], point) /
                          triangle_area;
  if (barycentric_coords[0] < 0) {
    return false;
  }
  barycentric_coords[1] = edge_function_2D(raster_space_triangle[2],
                                           raster_space_triangle[0], point) /
                          triangle_area;
  if (barycentric_coords[1] < 0) {
    return false;
  }
  barycentric_coords[2] = edge_function_2D(raster_space_triangle[0],
                                           raster_space_triangle[1], point) /
                          triangle_area;
  if (barycentric_coords[2] < 0) {
    return false;
  }

  // ADD RULES AGAINST COMMON EDGES

  return true;
}

void Rasterizer::vertex_program(Buffer<int, char>& frame_buffer,
                                Buffer<int, float>& depth_buffer,
                                Tensor& projection_matrix, Camera& camera,
                                std::vector<Mesh*>& meshes,
                                std::vector<Light*>& lights) {
  // get totatl num triangles
  int num_triangles_total = 0;
  for (auto& mesh_ptr : meshes) {
    num_triangles_total += mesh_ptr->num_triangles();
  }

  std::vector<std::array<Vector<3>, 3>> world_space_triangles;
  world_space_triangles.reserve(num_triangles_total);
  std::vector<Vector<3>> face_normals;
  face_normals.reserve(num_triangles_total);
  for (auto& mesh_ptr : meshes) {
    Mesh& mesh = *mesh_ptr;
    int num_triangles = mesh.num_triangles();
    for (int triangle_index = 0; triangle_index < num_triangles;
         triangle_index++) {
      Vector<3>* local_triangle = &mesh.vertices_[3 * triangle_index];
      std::array<Vector<3>, 3> world_space_triangle =
          transform_triangle(local_triangle, mesh.local_to_world_mat_);
      world_space_triangles.push_back(world_space_triangle);
      Vector<3> triangle_plane_normal = get_plane_normal(world_space_triangle);
      face_normals.push_back(triangle_plane_normal);
    }
  }

  int triangle_offset = 0;
  for (auto& mesh_ptr : meshes) {
    Mesh& mesh = *mesh_ptr;
    auto normal_to_world_mat = invert_transpose(mesh.local_to_world_mat_);
    int num_triangles = mesh.num_triangles();
    for (int triangle_index = 0; triangle_index < num_triangles;
         triangle_index++) {
      Vector<3>* local_triangle = &mesh.vertices_[3 * triangle_index];
      std::array<Vector<3>, 3>& world_space_triangle =
          world_space_triangles[triangle_offset + triangle_index];
      std::array<Vector<3>, 3> world_normals = normals_to_world(
          &mesh.normals_[3 * triangle_index], normal_to_world_mat);

      if (is_backface(world_space_triangle, world_normals, camera)) {
        continue;
      }

      Vector<2>* local_texture_coords = &mesh.uvs_[3 * triangle_index];
      std::array<Vector<4>, 3> triangle_clipping_space =
          project_vertices(world_space_triangle, projection_matrix);
      std::vector<std::array<Vector<4>, 3>> clipped_triangles =
          clip_vertices(triangle_clipping_space);
      std::vector<std::array<Vector<3>, 3>> raster_space_triangles =
          transform_to_raster_space(clipped_triangles, frame_buffer.shape());
      for (auto& raster_space_triangle : raster_space_triangles) {
        fragment_program(frame_buffer, depth_buffer, camera,
                         world_space_triangle, raster_space_triangle,
                         world_normals, local_texture_coords, meshes, lights,
                         world_space_triangles, face_normals);
      }
    }
    triangle_offset += num_triangles;
  }
}

Tensor get_bounding_box(std::array<Vector<3>, 3>& raster_space_triangle,
                        const BoxShape<int>& image_shape) {
  auto x_coords = {raster_space_triangle[0][0], raster_space_triangle[1][0],
                   raster_space_triangle[2][0]};
  auto y_coords = {raster_space_triangle[0][1], raster_space_triangle[1][1],
                   raster_space_triangle[2][1]};
  auto top_left = {std::clamp(std::floor(std::min(y_coords)), 0.f,
                              (float)image_shape.height),
                   std::clamp(std::floor(std::min(x_coords)), 0.f,
                              (float)image_shape.width)};
  auto bottom_right = {std::clamp(std::ceil(std::max(y_coords)), 0.f,
                                  (float)image_shape.height + 1),
                       std::clamp(std::ceil(std::max(x_coords)), 0.f,
                                  (float)image_shape.width + 1)};
  return Tensor({
      top_left,
      bottom_right,
  });
}

std::vector<Vector<2>> get_sample_points(Vector<2>& pixel) {
  return {
      pixel + 0.5,
  };
}

float interpolate_z(std::array<float, 3>& barycentric_coords,
                    std::array<Vector<3>, 3>& raster_space_triangle) {
  float one_over_z = barycentric_coords[0] / raster_space_triangle[0][2] +
                     barycentric_coords[1] / raster_space_triangle[1][2] +
                     barycentric_coords[2] / raster_space_triangle[2][2];
  float z = 1 / one_over_z;
  return z;
}

bool depth_buffer_test(Buffer<int, float>& depth_buffer, int& buffer_offset,
                       float& z) {
  if (z < depth_buffer[buffer_offset]) {
    return true;
  }
  return false;
}

template <typename T>
T interpolate_vertex_attribute(T* attributes,
                               std::array<Vector<3>, 3>& raster_space_triangle,
                               std::array<float, 3>& barycentric_coords,
                               float z) {
  return (attributes[0] / raster_space_triangle[0][2] * barycentric_coords[0] +
          attributes[1] / raster_space_triangle[1][2] * barycentric_coords[1] +
          attributes[2] / raster_space_triangle[2][2] * barycentric_coords[2]) *
         z;
}

void Rasterizer::fragment_program(
    Buffer<int, char>& frame_buffer, Buffer<int, float>& depth_buffer,
    Camera& camera, std::array<Vector<3>, 3>& world_space_triangle,
    std::array<Vector<3>, 3>& raster_space_triangle,
    std::array<Vector<3>, 3>& world_normals, Vector<2>* vertex_texture_coords,
    std::vector<Mesh*>& meshes, std::vector<Light*>& lights,
    std::vector<std::array<Vector<3>, 3>>& world_space_triangles,
    std::vector<Vector<3>>& face_normals) {
  Tensor bounding_box =
      get_bounding_box(raster_space_triangle, frame_buffer.shape());
  auto& camera_mat = camera.local_to_world_mat_;
  Vector<3> camera_position =
      Vector<3>({camera_mat(3, 0), camera_mat(3, 1), camera_mat(3, 2)});
  std::array<float, 3> barycentric_coords;
  float triangle_area =
      edge_function_2D(raster_space_triangle[0], raster_space_triangle[1],
                       raster_space_triangle[2]);
  for (int y = 0; y < bounding_box[1][0] - bounding_box[0][0]; ++y) {
    for (int x = 0; x < bounding_box[1][1] - bounding_box[0][1]; ++x) {
      Vector<2> pixel({x + bounding_box[0][1], y + bounding_box[0][0]});
      int buffer_offset = pixel[1] * frame_buffer.width() + pixel[0];
      int frame_buffer_offset = 3 * buffer_offset;
      std::vector<Vector<2>> sample_points = get_sample_points(pixel);

      auto current_color =
          Vector<3>({(float)(u_int8_t)frame_buffer[frame_buffer_offset],
                     (float)(u_int8_t)frame_buffer[frame_buffer_offset + 1],
                     (float)(u_int8_t)frame_buffer[frame_buffer_offset + 2]});
      auto pixel_color = Vector<3>(0);
      int hit_counter = 0;
      float min_z = 1;

      for (auto& point : sample_points) {
        if (!is_inside_triangle_2D(point, raster_space_triangle, triangle_area,
                                   barycentric_coords)) {
          continue;
        }
        float z = interpolate_z(barycentric_coords, raster_space_triangle);
        min_z = std::min(min_z, z);
        if (!depth_buffer_test(depth_buffer, buffer_offset, z)) {
          continue;
        }

        hit_counter += 1;

        // compute uvs
        Vector<2> texture_coord = interpolate_vertex_attribute(
            vertex_texture_coords, raster_space_triangle, barycentric_coords,
            z);

        // compute view direction
        Vector<3> point_world_space = interpolate_vertex_attribute(
            world_space_triangle.begin(), raster_space_triangle,
            barycentric_coords, z);
        Vector<3> view_direction = camera_position - point_world_space;
        view_direction = view_direction.normalize();

        // compute normal
        Vector<3> point_normal_world_space = interpolate_vertex_attribute(
            world_normals.begin(), raster_space_triangle, barycentric_coords,
            z);

        pixel_color =
            pixel_color + shading_program(point_world_space, view_direction,
                                          point_normal_world_space,
                                          texture_coord, meshes, lights,
                                          world_space_triangles, face_normals) *
                              255;
      }

      if (hit_counter > 0) {
        // update depth buffer
        depth_buffer[buffer_offset] =
            std::min(depth_buffer[buffer_offset], min_z);

        pixel_color =
            pixel_color + current_color * (sample_points.size() - hit_counter);
        pixel_color = pixel_color / sample_points.size();

        frame_buffer[frame_buffer_offset] = pixel_color[0];
        frame_buffer[frame_buffer_offset + 1] = pixel_color[1];
        frame_buffer[frame_buffer_offset + 2] = pixel_color[2];
      }
    }
  }
}

Vector<3> clip_color(Vector<3> color) {
  float max_channel = std::max({color[0], color[1], color[2]});
  if (max_channel <= 1) {
    return color;
  }
  return color / max_channel;
}

Vector<3> Rasterizer::shading_program(
    Vector<3>& point_world_space, Vector<3>& view_direction,
    Vector<3>& point_normal_world_space, Vector<2>& texture_coord,
    std::vector<Mesh*>& meshes, std::vector<Light*>& lights,
    std::vector<std::array<Vector<3>, 3>>& world_space_triangles,
    std::vector<Vector<3>>& face_normals) {
  // simple tile texture (get albedo from texture)
  const int M = 10;
  float checker = (fmod(texture_coord[0] * M, 1.0) > 0.5) ^
                  (fmod(texture_coord[1] * M, 1.0) < 0.5);
  float c = 0.3 * (1 - checker) + 0.7 * checker;
  Vector<3> albedo(c);

  Vector<3> point_color(0);
  for (auto& light : lights) {
    point_color =
        point_color + light->shade(albedo, point_world_space, view_direction,
                                   point_normal_world_space, meshes,
                                   world_space_triangles, face_normals);
  }

  return clip_color(point_color);
}

void Rasterizer::render(Buffer<int, char>& frame_buffer, Camera& camera,
                        std::vector<Mesh*>& meshes,
                        std::vector<Light*>& lights) {
  auto depth_buffer = Buffer<int, float>(frame_buffer.shape(), 1);
  camera.set_image_shape(frame_buffer.shape());
  auto projection_matrix = camera.projection_matrix();
  Rasterizer::vertex_program(frame_buffer, depth_buffer, projection_matrix,
                             camera, meshes, lights);
}
