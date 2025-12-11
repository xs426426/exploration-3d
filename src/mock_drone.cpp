#include "mock_drone.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <random>

namespace exploration {

MockDrone::MockDrone()
    : environment_(new pcl::PointCloud<pcl::PointXYZRGB>()) {
}

MockDrone::~MockDrone() = default;

bool MockDrone::loadEnvironment(const std::string& pcdFile) {
    std::cout << "[MockDrone] 加载环境点云: " << pcdFile << std::endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcdFile, *environment_) == -1) {
        std::cerr << "[MockDrone] 无法加载 PCD 文件: " << pcdFile << std::endl;
        return false;
    }

    std::cout << "[MockDrone] 加载了 " << environment_->size() << " 个点" << std::endl;

    // 构建 KD-Tree
    kdtree_.setInputCloud(environment_);

    // 计算边界
    pcl::PointXYZRGB minPt, maxPt;
    pcl::getMinMax3D(*environment_, minPt, maxPt);
    envMinBound_ = {minPt.x, minPt.y, minPt.z};
    envMaxBound_ = {maxPt.x, maxPt.y, maxPt.z};

    std::cout << "[MockDrone] 环境边界: "
              << "X[" << envMinBound_.x << ", " << envMaxBound_.x << "] "
              << "Y[" << envMinBound_.y << ", " << envMaxBound_.y << "] "
              << "Z[" << envMinBound_.z << ", " << envMaxBound_.z << "]" << std::endl;

    environmentLoaded_ = true;
    return true;
}

void MockDrone::setSensorConfig(const SensorConfig& config) {
    sensorConfig_ = config;
}

void MockDrone::setKinematicsConfig(const DroneKinematicsConfig& config) {
    kinematicsConfig_ = config;
}

void MockDrone::setInitialPose(const Point3D& position, double yaw) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    state_.position = position;
    state_.yaw = yaw;
    state_.velocity = {0, 0, 0};
    state_.isMoving = false;
    targetPosition_ = position;
    targetYaw_ = yaw;
    hasTarget_ = false;

    std::cout << "[MockDrone] 初始位置设置为: ("
              << position.x << ", " << position.y << ", " << position.z
              << "), yaw=" << yaw * 180.0 / M_PI << "°" << std::endl;
}

DroneState MockDrone::getState() const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return state_;
}

Point3D MockDrone::getPosition() const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return state_.position;
}

double MockDrone::getYaw() const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return state_.yaw;
}

PointCloud MockDrone::perceive() {
    if (!environmentLoaded_) {
        std::cerr << "[MockDrone] 环境未加载!" << std::endl;
        return PointCloud();
    }

    return extractVisiblePoints();
}

void MockDrone::setTargetPosition(const Point3D& target) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    targetPosition_ = target;
    hasTarget_ = true;
    state_.isMoving = true;

    // 自动调整朝向朝向目标
    double dx = target.x - state_.position.x;
    double dy = target.y - state_.position.y;
    if (std::abs(dx) > 0.01 || std::abs(dy) > 0.01) {
        targetYaw_ = std::atan2(dy, dx);
    }
}

void MockDrone::setTargetYaw(double yaw) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    targetYaw_ = normalizeAngle(yaw);
}

void MockDrone::update(double dt) {
    std::lock_guard<std::mutex> lock(stateMutex_);

    if (!hasTarget_) {
        state_.isMoving = false;
        return;
    }

    // 计算到目标的距离
    double dx = targetPosition_.x - state_.position.x;
    double dy = targetPosition_.y - state_.position.y;
    double dz = targetPosition_.z - state_.position.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

    // 检查是否到达
    if (distance < kinematicsConfig_.positionTolerance) {
        state_.position = targetPosition_;
        state_.velocity = {0, 0, 0};
        state_.isMoving = false;
        hasTarget_ = false;
        return;
    }

    // 计算移动方向和速度
    double speed = std::min(kinematicsConfig_.maxVelocity, distance / dt);
    double vx = (dx / distance) * speed;
    double vy = (dy / distance) * speed;
    double vz = (dz / distance) * speed;

    // 更新位置
    state_.position.x += vx * dt;
    state_.position.y += vy * dt;
    state_.position.z += vz * dt;
    state_.velocity = {vx, vy, vz};

    // 更新偏航角 (平滑转向)
    double yawError = normalizeAngle(targetYaw_ - state_.yaw);
    double maxYawChange = kinematicsConfig_.maxYawRate * M_PI / 180.0 * dt;
    if (std::abs(yawError) > maxYawChange) {
        state_.yaw += (yawError > 0 ? maxYawChange : -maxYawChange);
    } else {
        state_.yaw = targetYaw_;
    }
    state_.yaw = normalizeAngle(state_.yaw);
}

bool MockDrone::hasReachedTarget() const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return !hasTarget_ && !state_.isMoving;
}

size_t MockDrone::getEnvironmentPointCount() const {
    return environment_ ? environment_->size() : 0;
}

void MockDrone::getEnvironmentBounds(Point3D& minBound, Point3D& maxBound) const {
    minBound = envMinBound_;
    maxBound = envMaxBound_;
}

bool MockDrone::checkCollision(const Point3D& position, double radius) const {
    if (!environmentLoaded_) return false;

    pcl::PointXYZRGB searchPoint;
    searchPoint.x = position.x;
    searchPoint.y = position.y;
    searchPoint.z = position.z;

    std::vector<int> indices;
    std::vector<float> distances;

    // 搜索半径内是否有点 (有点说明有障碍物)
    if (kdtree_.radiusSearch(searchPoint, radius, indices, distances) > 0) {
        return true;  // 碰撞
    }
    return false;
}

PointCloud MockDrone::extractVisiblePoints() {
    PointCloud result;
    result.timestamp = static_cast<uint64_t>(
        std::chrono::system_clock::now().time_since_epoch().count() / 1000000);

    Point3D dronePos;
    double droneYaw;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        dronePos = state_.position;
        droneYaw = state_.yaw;
    }

    // 使用 KD-Tree 搜索范围内的点
    pcl::PointXYZRGB searchPoint;
    searchPoint.x = dronePos.x;
    searchPoint.y = dronePos.y;
    searchPoint.z = dronePos.z;

    std::vector<int> indices;
    std::vector<float> distances;

    // 搜索最大感知距离内的所有点
    int numFound = kdtree_.radiusSearch(searchPoint, sensorConfig_.maxRange, indices, distances);

    // 调试信息
    static int debugCount = 0;
    if (debugCount++ < 5) {
        std::cout << "[MockDrone] perceive() - Drone at (" << dronePos.x << ", " << dronePos.y << ", " << dronePos.z
                  << "), yaw=" << droneYaw * 180.0 / M_PI << "°" << std::endl;
        std::cout << "[MockDrone] radiusSearch(range=" << sensorConfig_.maxRange << ") found " << numFound << " points" << std::endl;
    }

    // 视锥体参数
    double halfHFov = sensorConfig_.horizontalFov * M_PI / 360.0;  // 半角
    double halfVFov = sensorConfig_.verticalFov * M_PI / 360.0;

    // 筛选在视锥体内的点
    for (size_t i = 0; i < indices.size(); ++i) {
        const auto& pt = environment_->points[indices[i]];

        // 相对于无人机的位置
        double dx = pt.x - dronePos.x;
        double dy = pt.y - dronePos.y;
        double dz = pt.z - dronePos.z;

        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        // 跳过太近的点
        if (dist < sensorConfig_.minRange) continue;

        // 转换到无人机坐标系 (考虑 yaw)
        double cosYaw = std::cos(-droneYaw);
        double sinYaw = std::sin(-droneYaw);
        double localX = dx * cosYaw - dy * sinYaw;  // 前方
        double localY = dx * sinYaw + dy * cosYaw;  // 左侧
        double localZ = dz;                          // 上方

        // 只看前方的点 (localX > 0)
        if (localX <= 0) continue;

        // 检查水平视场角
        double hAngle = std::atan2(localY, localX);
        if (std::abs(hAngle) > halfHFov) continue;

        // 检查垂直视场角
        double vAngle = std::atan2(localZ, std::sqrt(localX*localX + localY*localY));
        if (std::abs(vAngle) > halfVFov) continue;

        // 通过视锥体检查，添加到结果
        result.points.push_back({pt.x, pt.y, pt.z});

        // 限制点数
        if (result.points.size() >= static_cast<size_t>(sensorConfig_.maxPointsPerFrame)) {
            break;
        }
    }

    return result;
}

double MockDrone::normalizeAngle(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

bool MockDrone::isInFrustum(const Point3D& point) const {
    // 这个函数在 extractVisiblePoints 中内联实现了
    return true;
}

} // namespace exploration
