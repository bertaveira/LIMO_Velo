cmake_minimum_required(VERSION 3.8)
project(limovelo_cpp LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

# Add PCL library
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
include(ProcessorCount)
ProcessorCount(N)
message("Processer number:  ${N}")
if(N GREATER 4)
add_definitions(-DMP_EN)
add_definitions(-DMP_PROC_NUM=3)
message("core for MP: 3")
elseif(N GREATER 3)
add_definitions(-DMP_EN)
add_definitions(-DMP_PROC_NUM=2)
message("core for MP: 2")
else()
add_definitions(-DMP_PROC_NUM=1)
endif()
else()
add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/external
    external/ikd-Tree/ikd_Tree
    external/IKFoM/IKFoM_toolkit
    external/IKFoM
    )
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/modules LIMO_VELO_MODULES_SRC)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/objects LIMO_VELO_OBJECTS_SRC)

add_library(limovelo_cpp)
add_library(limovelo::limovelo_cpp ALIAS limovelo_cpp)

target_sources(limovelo_cpp PRIVATE 
    src/Limovelo.cpp
    src/Params.cpp
    src/Utils.cpp
    ${LIMO_VELO_MODULES_SRC}
    ${LIMO_VELO_OBJECTS_SRC}
    
    external/ikd-Tree/ikd_Tree/ikd_Tree.cpp
    external/IKFoM/use-ikfom.cpp
)

target_link_libraries(limovelo_cpp PUBLIC Eigen3::Eigen ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)