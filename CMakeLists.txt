cmake_minimum_required(VERSION 3.16)
project(Exploration3D VERSION 1.0.0 LANGUAGES CXX)

# C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 编译选项
if(MSVC)
    add_compile_options(/W4 /utf-8)
else()
    add_compile_options(-Wall -Wextra -O2)
endif()

# ============================================================
# 依赖查找
# ============================================================

# OctoMap
find_package(octomap REQUIRED)
message(STATUS "Found OctoMap: ${OCTOMAP_INCLUDE_DIRS}")

# Protobuf
find_package(Protobuf REQUIRED)
message(STATUS "Found Protobuf: ${Protobuf_VERSION}")

# Eclipse Paho MQTT C++
find_package(PahoMqttCpp REQUIRED)
message(STATUS "Found Paho MQTT C++")

# Eigen3 (用于数学计算)
find_package(Eigen3 REQUIRED)
message(STATUS "Found Eigen3: ${EIGEN3_INCLUDE_DIR}")

# PCL (用于点云处理)
find_package(PCL 1.10 REQUIRED COMPONENTS common io kdtree)
message(STATUS "Found PCL: ${PCL_VERSION}")

# yaml-cpp (用于配置文件)
find_package(yaml-cpp REQUIRED)
message(STATUS "Found yaml-cpp: ${YAML_CPP_VERSION}")

# OpenMP (可选，用于并行加速)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    message(STATUS "Found OpenMP: ${OpenMP_CXX_VERSION}")
endif()

# ============================================================
# Protobuf 生成
# ============================================================

set(PROTO_FILES
    ${CMAKE_SOURCE_DIR}/proto/drone_message.proto
)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# ============================================================
# 源文件
# ============================================================

set(SOURCES
    src/main.cpp
    src/mqtt_client.cpp
    src/octomap_manager.cpp
    src/frontier_detector.cpp
    src/path_planner.cpp
    src/exploration_engine.cpp
    src/config_loader.cpp
    ${PROTO_SRCS}
)

set(HEADERS
    include/mqtt_client.hpp
    include/octomap_manager.hpp
    include/frontier_detector.hpp
    include/path_planner.hpp
    include/exploration_engine.hpp
    include/config_loader.hpp
    include/types.hpp
    ${PROTO_HDRS}
)

# ============================================================
# 可执行文件
# ============================================================

add_executable(exploration_3d ${SOURCES} ${HEADERS})

target_include_directories(exploration_3d PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}  # Protobuf 生成的头文件
    ${OCTOMAP_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Protobuf_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(exploration_3d PRIVATE
    ${OCTOMAP_LIBRARIES}
    ${Protobuf_LIBRARIES}
    PahoMqttCpp::paho-mqttpp3
    Eigen3::Eigen
    yaml-cpp
    ${PCL_LIBRARIES}
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(exploration_3d PRIVATE OpenMP::OpenMP_CXX)
endif()

# ============================================================
# 仿真测试程序 (不依赖 MQTT)
# ============================================================

set(SIM_SOURCES
    src/sim_test.cpp
    src/mock_drone.cpp
    src/octomap_manager.cpp
    src/frontier_detector.cpp
    src/path_planner.cpp
)

add_executable(sim_test ${SIM_SOURCES})

target_include_directories(sim_test PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${OCTOMAP_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(sim_test PRIVATE
    ${OCTOMAP_LIBRARIES}
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(sim_test PRIVATE OpenMP::OpenMP_CXX)
endif()

# ============================================================
# 安装
# ============================================================

install(TARGETS exploration_3d
    RUNTIME DESTINATION bin
)

install(FILES config/exploration_config.yaml
    DESTINATION config
)

# ============================================================
# 测试 (可选)
# ============================================================

option(BUILD_TESTS "Build unit tests" OFF)

if(BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)

    add_executable(test_octomap tests/test_octomap.cpp)
    target_link_libraries(test_octomap PRIVATE
        ${OCTOMAP_LIBRARIES}
        GTest::gtest_main
    )

    add_test(NAME OctoMapTest COMMAND test_octomap)
endif()
