#pragma once

#include <string>
#include <vector>

#include "../../geo_lib/vector.h"

bool loadOBJ(const std::string path, std::vector<Vector<3>>& vertices,
             std::vector<Vector<2>>& uvs, std::vector<Vector<3>>& normals);