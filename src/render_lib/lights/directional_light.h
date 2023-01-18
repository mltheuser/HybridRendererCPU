#include "light.h"

class DirectionalLight : public Light {
 public:
  DirectionalLight(Vector<3> direction, Vector<3> color, float intensity, bool cast_shadow=true);
  Vector<3> virtual shade(
      Vector<3>& albedo, Vector<3>& point_world_space,
      Vector<3>& view_direction, Vector<3>& point_normal_world_space,
      std::vector<Mesh*>& meshes,
      std::vector<std::array<Vector<3>, 3>>& world_space_triangles,
      std::vector<Vector<3>>& face_normals);
  void set_direction(Vector<3> direction);
  Vector<3> direction();
  Vector<3> color_;
  float intensity_;
  bool cast_shadow_;
};