cmake_minimum_required(VERSION 3.5)
project(fcl_self_collision_checker)


find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(fcl REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(resource_retriever REQUIRED)

add_executable(collision_checker src/main.cpp)
set_target_properties(collision_checker PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "$ORIGIN;/opt/ros/foxy/lib"
)

target_include_directories(collision_checker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


# Prefer imported target if provided by fcl; fallback to plain 'fcl'
if(TARGET fcl::fcl)
  set(FCL_TARGET fcl::fcl)
else()
  set(FCL_TARGET fcl)
endif()

target_link_libraries(collision_checker ${FCL_TARGET} Eigen3::Eigen)


ament_target_dependencies(
  collision_checker
  rclcpp
  urdfdom
  sensor_msgs
  urdf
  ament_index_cpp
  visualization_msgs
  kdl_parser
  tf2_msgs
  tf2
  tf2_ros
  tf2_eigen
  resource_retriever
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS collision_checker
  DESTINATION lib/${PROJECT_NAME})

ament_package()