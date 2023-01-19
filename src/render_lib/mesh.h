#pragma once

#include "inhabitant.h"

#include <string>

class Mesh : public Inhabitant {
 public:
  Mesh(std::vector<Vector<3> > vertices, std::vector<Vector<2> > uvs,
       std::vector<Vector<3> > normals);
  Mesh(const std::string& obj_path);

  size_t num_triangles() const;

  std::vector<Vector<3> > vertices_;
  std::vector<Vector<2> > uvs_;
  std::vector<Vector<3> > normals_;
};