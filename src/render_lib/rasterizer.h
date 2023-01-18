#pragma once

#include "camera.h"
#include "helpers/buffer.h"
#include "lights/directional_light.h"
#include "mesh.h"

namespace Rasterizer {
void vertex_program(Buffer<int, char>& frame_buffer,
                    Buffer<int, float>& depth_buffer, Tensor& projection_matrix,
                    Camera& camera, std::vector<Mesh*>& meshes,
                    std::vector<Light*>& lights);

void fragment_program(
    Buffer<int, char>& frame_buffer, Buffer<int, float>& depth_buffer,
    Camera& camera, std::array<Vector<3>, 3>& world_space_triangle,
    std::array<Vector<3>, 3>& raster_space_triangle,
    std::array<Vector<3>, 3>& world_normals, Vector<2>* vertex_texture_coords,
    std::vector<Mesh*>& meshes, std::vector<Light*>& lights,
    std::vector<std::array<Vector<3>, 3>>& world_space_triangles,
    std::vector<Vector<3>>& face_normals);

Vector<3> shading_program(
    Vector<3>& point_world_space, Vector<3>& view_direction,
    Vector<3>& point_normal_world_space, Vector<2>& texture_coord,
    std::vector<Mesh*>& meshes, std::vector<Light*>& lights,
    std::vector<std::array<Vector<3>, 3>>& world_space_triangles,
    std::vector<Vector<3>>& face_normals);

void render(Buffer<int, char>& frame_buffer, Camera& camera,
            std::vector<Mesh*>& meshes, std::vector<Light*>& lights);
};  // namespace Rasterizer