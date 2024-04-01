cmake_minimum_required(VERSION 3.8)
project(limovelo LANGUAGES CXX)

set(ignore ${CATKIN_INSTALL_INTO_PREFIX_ROOT})
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/limovelo_cpp)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(PCL 1.8 REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  include
)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_LIST)
add_executable(limovelo ${SRC_LIST})
ament_target_dependencies(
  limovelo 
  geometry_msgs
  nav_msgs
  sensor_msgs
  rclcpp
  std_msgs
  pcl_ros
  tf2
  tf2_ros
  PCL
)

target_link_libraries(limovelo ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} limovelo::limovelo_cpp)
target_include_directories(limovelo
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../cpp/limovelo_cpp/include
  PRIVATE ${PYTHON_INCLUDE_DIRS}
)

install(
  TARGETS
  limovelo
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  set(ament_cmake_copyright_FOUND TRUE)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()