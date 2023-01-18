#include "mesh.h"

#include "./helpers/simple_obj_loader.h"

Mesh::Mesh(const std::string& obj_path) {
    loadOBJ(obj_path, vertices_, uvs_, normals_);
}

size_t Mesh::num_triangles() const { return (size_t) vertices_.size() / 3; }
