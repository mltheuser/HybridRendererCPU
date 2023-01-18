#include "simple_obj_loader.h"

#include <string.h>

bool loadOBJ(const std::string path, std::vector<Vector<3>>& vertices,
             std::vector<Vector<2>>& uvs, std::vector<Vector<3>>& normals) {
  FILE* file = fopen(path.c_str(), "r");
  if (file == NULL) {
    printf("Can not open file.\n");
    return false;
  }

  std::vector< unsigned int > vertexIndices, uvIndices, normalIndices;
  std::vector< Vector<3> > temp_vertices;
  std::vector< Vector<2> > temp_uvs;
  std::vector< Vector<3> > temp_normals;

  while (1) {
    char lineHeader[128];
    // read the first word of the line
    int res = fscanf(file, "%s", lineHeader);
    if (res == EOF) break;  // EOF = End Of File. Quit the loop.

    // else : parse lineHeader
    if (strcmp(lineHeader, "v") == 0) {
      Vector<3> vertex(0);
      fscanf(file, "%f %f %f\n", &vertex[0], &vertex[1], &vertex[2]);
      temp_vertices.push_back(vertex);
    } else if (strcmp(lineHeader, "vt") == 0) {
      Vector<2> uv(0);
      fscanf(file, "%f %f\n", &uv[0], &uv[1]);
      temp_uvs.push_back(uv);
    } else if (strcmp(lineHeader, "vn") == 0) {
      Vector<3> normal(0);
      fscanf(file, "%f %f %f\n", &normal[0], &normal[1], &normal[2]);
      temp_normals.push_back(normal);
    } else if (strcmp(lineHeader, "f") == 0) {
      std::string vertex1, vertex2, vertex3;
      unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
      int matches = fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n",
                           &vertexIndex[0], &uvIndex[0], &normalIndex[0],
                           &vertexIndex[1], &uvIndex[1], &normalIndex[1],
                           &vertexIndex[2], &uvIndex[2], &normalIndex[2]);
      if (matches != 9) {
        printf(
            "File can't be read.");
        return false;
      }
      vertexIndices.push_back(vertexIndex[0]);
      vertexIndices.push_back(vertexIndex[1]);
      vertexIndices.push_back(vertexIndex[2]);
      uvIndices.push_back(uvIndex[0]);
      uvIndices.push_back(uvIndex[1]);
      uvIndices.push_back(uvIndex[2]);
      normalIndices.push_back(normalIndex[0]);
      normalIndices.push_back(normalIndex[1]);
      normalIndices.push_back(normalIndex[2]);
    }
  }

  for( unsigned int i=0; i<vertexIndices.size(); i++ ){
    unsigned int vertexIndex = vertexIndices[i];
    Vector<3> vertex = temp_vertices[ vertexIndex-1 ];
    vertices.push_back(vertex);
  }

  for( unsigned int i=0; i<uvIndices.size(); i++ ){
    unsigned int uvIndex = uvIndices[i];
    Vector<2> uv = temp_uvs[ uvIndex-1 ];
    uvs.push_back(uv);
  }

  for( unsigned int i=0; i<normalIndices.size(); i++ ){
    unsigned int normalIndex = normalIndices[i];
    Vector<3> normal = temp_normals[ normalIndex-1 ];
    normals.push_back(normal);
  }

  return true;

}