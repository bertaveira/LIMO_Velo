cmake_minimum_required(VERSION 2.8.12)
project(LimoVeloPy)

find_package(PCL REQUIRED)

add_subdirectory(pybind11)
add_subdirectory(limovelo_cpp)
pybind11_add_module(limovelo_py src/main.cpp)
target_include_directories(limovelo_py PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/limovelo_cpp/include)
target_link_libraries(limovelo_py PRIVATE limovelo::limovelo_cpp ${PCL_LIBRARIES})
