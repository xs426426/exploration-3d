#pragma once

#include "types.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>

namespace exploration {

// 传感器配置
struct SensorConfig {
    double horizontalFov = 60.0;   // 水平视场角 (度)
    double verticalFov = 45.0;     // 垂直视场角 (度)
    double maxRange = 5.0;         // 最大感知距离 (米)
    double minRange = 0.1;         // 最小感知距离 (米)
    int maxPointsPerFrame = 10000; // 每帧最大点数
};

// 无人机运动学配置
struct DroneKinematicsConfig {
    double maxVelocity = 1.0;      // 最大速度 (m/s)
    double maxYawRate = 45.0;      // 最大偏航角速度 (度/s)
    double positionTolerance = 0.1; // 位置到达容差 (米)
};

// 无人机状态
struct DroneState {
    Point3D position{0, 0, 1.0};   // 当前位置
    double yaw = 0.0;              // 偏航角 (弧度)
    Point3D velocity{0, 0, 0};     // 当前速度
    bool isMoving = false;         // 是否在移动
};

/**
 * Mock 无人机仿真器
 * 加载真实 PCD 点云，模拟无人机感知和运动
 */
class MockDrone {
public:
    MockDrone();
    ~MockDrone();

    // 初始化
    bool loadEnvironment(const std::string& pcdFile);
    void setSensorConfig(const SensorConfig& config);
    void setKinematicsConfig(const DroneKinematicsConfig& config);

    // 设置初始位置
    void setInitialPose(const Point3D& position, double yaw = 0.0);

    // 获取当前状态
    DroneState getState() const;
    Point3D getPosition() const;
    double getYaw() const;

    // 感知：获取当前视野内的点云
    PointCloud perceive();

    // 控制：设置目标位置
    void setTargetPosition(const Point3D& target);
    void setTargetYaw(double yaw);

    // 运动学更新 (调用频率决定仿真步长)
    void update(double dt);

    // 检查是否到达目标
    bool hasReachedTarget() const;

    // 获取环境信息
    size_t getEnvironmentPointCount() const;
    void getEnvironmentBounds(Point3D& minBound, Point3D& maxBound) const;

    // 碰撞检测
    bool checkCollision(const Point3D& position, double radius = 0.3) const;

private:
    // 视锥体裁剪：提取视野内的点
    PointCloud extractVisiblePoints();

    // 简单遮挡剔除 (可选)
    PointCloud removeOccludedPoints(const PointCloud& cloud);

    // 角度归一化到 [-pi, pi]
    double normalizeAngle(double angle) const;

    // 检查点是否在视锥体内
    bool isInFrustum(const Point3D& point) const;

private:
    // 环境点云 (Ground Truth)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr environment_;
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree_;
    bool environmentLoaded_ = false;

    // 环境边界
    Point3D envMinBound_, envMaxBound_;

    // 无人机状态
    DroneState state_;
    Point3D targetPosition_;
    double targetYaw_ = 0.0;
    bool hasTarget_ = false;

    // 配置
    SensorConfig sensorConfig_;
    DroneKinematicsConfig kinematicsConfig_;

    // 线程安全
    mutable std::mutex stateMutex_;
};

} // namespace exploration
