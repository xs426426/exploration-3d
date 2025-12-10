#include "exploration_engine.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <random>

namespace exploration {

ExplorationEngine::ExplorationEngine(const ExplorationConfig& config)
    : config_(config)
{
    // 创建各模块
    octomapManager_ = std::make_unique<OctoMapManager>(config_.resolution);
    frontierDetector_ = std::make_unique<FrontierDetector>(config_);
    pathPlanner_ = std::make_unique<PathPlanner>(config_);
    mqttClient_ = std::make_unique<MqttClient>(config_);

    // 根据配置设置路径模式
    if (config_.pathPlanningMode == "astar") {
        pathMode_ = PathMode::ASTAR;
    } else {
        pathMode_ = PathMode::DIRECT;
    }

    std::cout << "[ExplorationEngine] 初始化完成" << std::endl;
}

ExplorationEngine::~ExplorationEngine() {
    stop();
}

bool ExplorationEngine::initialize() {
    std::cout << "[ExplorationEngine] 正在初始化..." << std::endl;

    // 连接MQTT
    if (!mqttClient_->connect()) {
        std::cerr << "[ExplorationEngine] MQTT连接失败" << std::endl;
        return false;
    }

    // 设置回调
    mqttClient_->setPointCloudCallback(
        [this](const PointCloud& cloud) {
            this->onPointCloud(cloud);
        }
    );

    mqttClient_->setOdometryCallback(
        [this](const Pose& pose) {
            this->onOdometry(pose);
        }
    );

    std::cout << "[ExplorationEngine] 初始化成功" << std::endl;
    return true;
}

bool ExplorationEngine::start() {
    if (state_ == State::EXPLORING || state_ == State::NAVIGATING) {
        std::cout << "[ExplorationEngine] 已在探索中" << std::endl;
        return false;
    }

    std::cout << "[ExplorationEngine] 开始探索" << std::endl;

    state_ = State::EXPLORING;
    running_ = true;
    paused_ = false;

    stats_ = Statistics();
    stats_.startTime = std::chrono::steady_clock::now();

    lastUpdateTime_ = std::chrono::steady_clock::now();
    lastGoalTime_ = std::chrono::steady_clock::now();

    return true;
}

void ExplorationEngine::pause() {
    if (state_ == State::EXPLORING || state_ == State::NAVIGATING) {
        std::cout << "[ExplorationEngine] 暂停探索" << std::endl;
        paused_ = true;
        state_ = State::PAUSED;

        // 暂停当前任务
        if (!currentMissionId_.empty()) {
            mqttClient_->publishExecution(currentMissionId_, 1);  // PAUSE
        }
    }
}

void ExplorationEngine::resume() {
    if (state_ == State::PAUSED) {
        std::cout << "[ExplorationEngine] 恢复探索" << std::endl;
        paused_ = false;
        state_ = State::EXPLORING;

        // 恢复当前任务
        if (!currentMissionId_.empty()) {
            mqttClient_->publishExecution(currentMissionId_, 2);  // RESUME
        }
    }
}

void ExplorationEngine::stop() {
    if (running_) {
        std::cout << "[ExplorationEngine] 停止探索" << std::endl;

        running_ = false;
        paused_ = false;

        // 停止当前任务
        if (!currentMissionId_.empty()) {
            mqttClient_->publishExecution(currentMissionId_, 3);  // STOP
        }

        state_ = State::IDLE;
    }
}

void ExplorationEngine::run() {
    std::cout << "[ExplorationEngine] 进入主循环" << std::endl;

    while (running_) {
        update();

        // 控制更新频率
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(1000.0 / config_.updateRate))
        );
    }

    std::cout << "[ExplorationEngine] 退出主循环" << std::endl;
}

void ExplorationEngine::update() {
    if (!running_ || paused_) return;

    auto now = std::chrono::steady_clock::now();

    // 更新统计
    updateStatistics();

    // 状态机
    switch (state_) {
        case State::EXPLORING: {
            // 检查是否需要选择新目标
            auto timeSinceLastGoal = std::chrono::duration_cast<std::chrono::seconds>(
                now - lastGoalTime_).count();

            if (!hasValidGoal_ || timeSinceLastGoal > config_.goalTimeout) {
                if (!selectNextGoal()) {
                    if (isExplorationComplete()) {
                        state_ = State::COMPLETED;
                        std::cout << "[ExplorationEngine] 探索完成!" << std::endl;
                        printStatus();
                    }
                }
            }
            break;
        }

        case State::NAVIGATING: {
            // 检查是否到达目标
            if (hasReachedGoal()) {
                std::cout << "[ExplorationEngine] 到达目标点" << std::endl;
                frontierDetector_->addVisitedGoal(currentGoal_);
                hasValidGoal_ = false;
                state_ = State::EXPLORING;
                stats_.waypointsVisited++;
            }
            break;
        }

        case State::COMPLETED: {
            // 探索完成，停止运行
            running_ = false;
            break;
        }

        default:
            break;
    }

    lastUpdateTime_ = now;
}

void ExplorationEngine::onPointCloud(const PointCloud& cloud) {
    if (!running_) return;

    // 使用当前位姿作为传感器原点
    Point3D sensorOrigin = currentPose_.position;

    // 插入点云到OctoMap
    octomapManager_->insertPointCloud(cloud, sensorOrigin);
}

void ExplorationEngine::onOdometry(const Pose& pose) {
    // 计算移动距离
    if (running_) {
        double dist = currentPose_.position.distanceTo(pose.position);
        if (dist > 0.01 && dist < 10.0) {  // 过滤异常值
            stats_.totalDistance += dist;
        }
    }

    currentPose_ = pose;
}

bool ExplorationEngine::selectNextGoal() {
    std::cout << "[ExplorationEngine] 正在选择下一个目标..." << std::endl;

    // 检测前沿点
    auto frontiers = frontierDetector_->detectFrontiers(
        *octomapManager_,
        currentPose_.position
    );

    stats_.frontiersDetected = static_cast<int>(frontiers.size());

    if (frontiers.empty()) {
        std::cout << "[ExplorationEngine] 没有检测到前沿点" << std::endl;
        return false;
    }

    // 计算上一个方向（用于一致性奖励）
    Point3D lastDirection;
    Point3D* lastDirPtr = nullptr;
    if (hasValidGoal_) {
        lastDirection.x = currentGoal_.x - currentPose_.position.x;
        lastDirection.y = currentGoal_.y - currentPose_.position.y;
        lastDirection.z = currentGoal_.z - currentPose_.position.z;
        lastDirPtr = &lastDirection;
    }

    // 选择最优前沿
    FrontierCluster bestFrontier = frontierDetector_->selectBestFrontier(
        frontiers,
        currentPose_.position,
        lastDirPtr
    );

    if (bestFrontier.cells.empty()) {
        std::cout << "[ExplorationEngine] 没有可用的前沿点" << std::endl;
        return false;
    }

    // 规划路径
    if (!planPath(bestFrontier.centroid)) {
        handleUnreachableGoal();
        return false;
    }

    // 发布任务
    if (!publishMission(currentPath_)) {
        return false;
    }

    currentGoal_ = bestFrontier.centroid;
    hasValidGoal_ = true;
    lastGoalTime_ = std::chrono::steady_clock::now();
    state_ = State::NAVIGATING;

    std::cout << "[ExplorationEngine] 新目标: ("
              << currentGoal_.x << ", " << currentGoal_.y << ", " << currentGoal_.z
              << ")" << std::endl;

    return true;
}

bool ExplorationEngine::planPath(const Point3D& goal) {
    stats_.pathsPlanned++;

    if (pathMode_ == PathMode::DIRECT) {
        // 直线模式：检查直线路径是否可行
        bool clear = octomapManager_->isPathClear(
            currentPose_.position,
            goal,
            config_.resolution,
            config_.safetyMargin
        );

        if (!clear) {
            std::cout << "[ExplorationEngine] 直线路径不可行" << std::endl;
            return false;
        }

        currentPath_.waypoints.clear();
        currentPath_.waypoints.push_back(currentPose_.position);
        currentPath_.waypoints.push_back(goal);
        currentPath_.totalLength = currentPose_.position.distanceTo(goal);
        currentPath_.isValid = true;

    } else {
        // A*模式
        currentPath_ = pathPlanner_->planPath(
            currentPose_.position,
            goal,
            *octomapManager_
        );

        if (!currentPath_.isValid) {
            std::cout << "[ExplorationEngine] A*路径规划失败" << std::endl;
            return false;
        }

        // 简化路径
        currentPath_ = pathPlanner_->simplifyPath(currentPath_, *octomapManager_);
    }

    std::cout << "[ExplorationEngine] 路径规划成功, 路点数="
              << currentPath_.waypoints.size()
              << ", 长度=" << currentPath_.totalLength << "m" << std::endl;

    return true;
}

bool ExplorationEngine::publishMission(const Path3D& path) {
    Mission mission;
    mission.id = generateMissionId();
    mission.yaw = currentPose_.yaw;

    // 转换路点（跳过起点）
    for (size_t i = 1; i < path.waypoints.size(); ++i) {
        mission.waypoints.push_back(path.waypoints[i]);
    }

    if (mission.waypoints.empty()) {
        return false;
    }

    // 发布任务
    if (!mqttClient_->publishMission(mission)) {
        return false;
    }

    // 启动任务
    if (!mqttClient_->publishExecution(mission.id, 0)) {  // START
        return false;
    }

    currentMissionId_ = mission.id;
    return true;
}

bool ExplorationEngine::hasReachedGoal() const {
    if (!hasValidGoal_) return false;

    double dist = currentPose_.position.distanceTo(currentGoal_);
    return dist < config_.goalTolerance;
}

bool ExplorationEngine::isExplorationComplete() const {
    // 检测前沿点
    auto frontiers = octomapManager_->detectFrontierCells();

    // 过滤高度范围内的前沿点
    int validFrontiers = 0;
    for (const auto& f : frontiers) {
        if (f.z >= config_.minHeight && f.z <= config_.maxHeight) {
            double dist = f.distanceTo(currentPose_.position);
            if (dist <= config_.maxExplorationDistance) {
                validFrontiers++;
            }
        }
    }

    return validFrontiers == 0;
}

void ExplorationEngine::handleUnreachableGoal() {
    if (hasValidGoal_) {
        frontierDetector_->addUnreachableGoal(currentGoal_);
        stats_.unreachableGoals++;
        hasValidGoal_ = false;
    }
}

std::string ExplorationEngine::generateMissionId() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dis(10000, 99999);

    std::stringstream ss;
    ss << "exp_" << std::setfill('0') << std::setw(5) << dis(gen);
    return ss.str();
}

void ExplorationEngine::updateStatistics() {
    auto now = std::chrono::steady_clock::now();
    stats_.elapsedSeconds = std::chrono::duration<double>(now - stats_.startTime).count();
    stats_.exploredVolume = octomapManager_->getExploredVolume();
}

void ExplorationEngine::printStatus() const {
    std::cout << "\n========== 探索统计 ==========" << std::endl;
    std::cout << "已探索体积: " << std::fixed << std::setprecision(2)
              << stats_.exploredVolume << " m³" << std::endl;
    std::cout << "访问路点数: " << stats_.waypointsVisited << std::endl;
    std::cout << "规划路径数: " << stats_.pathsPlanned << std::endl;
    std::cout << "不可达目标: " << stats_.unreachableGoals << std::endl;
    std::cout << "飞行距离: " << std::fixed << std::setprecision(2)
              << stats_.totalDistance << " m" << std::endl;
    std::cout << "运行时间: " << std::fixed << std::setprecision(1)
              << stats_.elapsedSeconds << " s" << std::endl;

    auto mapStats = octomapManager_->getStats();
    std::cout << "地图节点: " << mapStats.totalNodes
              << " (占据=" << mapStats.occupiedNodes
              << ", 空闲=" << mapStats.freeNodes << ")" << std::endl;
    std::cout << "内存使用: " << std::fixed << std::setprecision(2)
              << mapStats.memoryUsage << " MB" << std::endl;
    std::cout << "==============================\n" << std::endl;
}

}  // namespace exploration
