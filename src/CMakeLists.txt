include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include_directories(
    geo_lib
    render_lib
)

add_library(geo_lib STATIC geo_lib/tensor.cc geo_lib/helpers.cc)

add_library(render_lib STATIC 
render_lib/inhabitant.cc
render_lib/camera.cc
render_lib/rasterizer.cc
render_lib/mesh.cc
render_lib/lights/directional_light.cc
render_lib/helpers/simple_obj_loader.cc
render_lib/helpers/helpers.cc
render_lib/raytracer.cc
)

add_executable(graphic main.cpp)

target_link_libraries(render_lib geo_lib)
target_link_libraries(graphic render_lib)


enable_testing()

add_executable(
  geo_lib_tests
  tests/geo_lib/tensor_test.cc
  tests/geo_lib/vector_test.cc
  tests/geo_lib/helpers_test.cc
)
target_link_libraries(
  geo_lib_tests
  GTest::gtest_main
  geo_lib
)

include(GoogleTest)
gtest_discover_tests(geo_lib_tests)