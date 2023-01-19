#include <math.h>

#include <chrono>
#include <fstream>
#include <iostream>

#include "geo_lib/helpers.h"
#include "mesh.h"
#include "rasterizer.h"

bool to_image(Buffer<int, char>& frame_buffer, const std::string& path) {
  FILE* imageFile;
  int x, y, pixel, height = frame_buffer.height(), width = frame_buffer.width();

  imageFile = fopen(path.c_str(), "w+");
  if (imageFile == NULL) {
    perror("ERROR: Cannot open output file");
    exit(EXIT_FAILURE);
  }

  fprintf(imageFile, "P6\n");                    // P6 filetype
  fprintf(imageFile, "%d %d\n", width, height);  // dimensions
  fprintf(imageFile, "255\n");                   // Max pixel

  fwrite(frame_buffer, 1, width * height * 3, imageFile);
  fclose(imageFile);

  return true;
}

int main() {
  Mesh cube = Mesh("../../src/models/bunny.obj");
  cube.scale({20, 20, 20});
  cube.translate({0, 0, -20});
  cube.rotate(0, 45, 0);

  Mesh ground_plane = Mesh("../../src/models/cube.obj");
  ground_plane.scale({2.5, 0.1, 2.5});
  ground_plane.translate({0, 2.4, -20});

  DirectionalLight dir_light = DirectionalLight(
      Vector<3>({-0.5, -1.0, 0}), Vector<3>({1.0, 0.5, 0.0}), 40.0, true);
  DirectionalLight dir_light_2 = DirectionalLight(
      Vector<3>({0.5, -1.0, 0}), Vector<3>({0.0, 0.5, 1.0}), 50.0, true);
  std::vector<Light*> lights = {&dir_light, &dir_light_2};

  Camera camera = Camera();

  BoxShape<int> image_shape = {512, 512, 3};
  Buffer<int, char> frame_buffer(image_shape, 0);
  std::vector<Mesh*> meshes = {&cube, &ground_plane};

  // create spinning animation
  int num_frames_for_rotation = 3 * 25;
  float rot_per_frame = 360.f / num_frames_for_rotation;
  for (int i = 0; i < num_frames_for_rotation; ++i) {
    frame_buffer.reset();
    auto t_start = std::chrono::high_resolution_clock::now();
    Rasterizer::render(frame_buffer, camera, meshes, lights);
    auto t_end = std::chrono::high_resolution_clock::now();
    float rotation = rot_per_frame * (i + 1);
    cube.rotate(0, -rot_per_frame, 0);
    float translation = sin(degrees_to_radians(2 * rotation));
    cube.translate(
        {0, -cube.local_to_world_mat_(3, 1) + 0.3f * translation, 0});
    std::string frame_name = "../../src/image" + std::to_string(i) + ".ppm";
    to_image(frame_buffer, frame_name);

    auto s_int = duration_cast<std::chrono::seconds>(t_end - t_start);
    std::cout << s_int.count() << "s\n";
  }
}
