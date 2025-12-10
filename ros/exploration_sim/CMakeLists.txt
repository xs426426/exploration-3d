cmake_minimum_required(VERSION 3.0.2)
project(exploration_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  sensor_msgs
  octomap_msgs
  octomap_ros
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
)

find_package(octomap REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp std_msgs geometry_msgs visualization_msgs
    nav_msgs sensor_msgs octomap_msgs octomap_ros
    tf2 tf2_ros pcl_ros pcl_conversions
)

# exploration-3d 库的路径
set(EXPLORATION_3D_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../..)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${EXPLORATION_3D_DIR}/include
)

# 源文件
set(EXPLORATION_SOURCES
  ${EXPLORATION_3D_DIR}/src/octomap_manager.cpp
  ${EXPLORATION_3D_DIR}/src/frontier_detector.cpp
  ${EXPLORATION_3D_DIR}/src/path_planner.cpp
  ${EXPLORATION_3D_DIR}/src/mock_drone.cpp
)

# 仿真节点
add_executable(sim_node
  src/sim_node.cpp
  ${EXPLORATION_SOURCES}
)

target_link_libraries(sim_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

# 安装
install(TARGETS sim_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
