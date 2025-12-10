#pragma once

#include "types.hpp"
#include "octomap_manager.hpp"
#include "frontier_detector.hpp"
#include "path_planner.hpp"
#include "mqtt_client.hpp"
#include <memory>
#include <atomic>
#include <thread>
#include <chrono>

namespace exploration {

/**
 * 3D 自主探索引擎
 * 协调所有模块完成自主探索任务
 */
class ExplorationEngine {
public:
    // 探索状态
    enum class State {
        IDLE,           // 空闲
        EXPLORING,      // 探索中
        NAVIGATING,     // 导航到目标
        PAUSED,         // 暂停
        COMPLETED,      // 探索完成
        ERROR           // 错误状态
    };

    // 路径规划模式
    enum class PathMode {
        DIRECT,         // 直线模式
        ASTAR           // A*路径搜索
    };

    explicit ExplorationEngine(const ExplorationConfig& config);
    ~ExplorationEngine();

    // 禁止拷贝
    ExplorationEngine(const ExplorationEngine&) = delete;
    ExplorationEngine& operator=(const ExplorationEngine&) = delete;

    // ============================================================
    // 生命周期
    // ============================================================

    /**
     * 初始化引擎（连接MQTT等）
     */
    bool initialize();

    /**
     * 启动探索
     */
    bool start();

    /**
     * 暂停探索
     */
    void pause();

    /**
     * 恢复探索
     */
    void resume();

    /**
     * 停止探索
     */
    void stop();

    /**
     * 运行主循环（阻塞）
     */
    void run();

    /**
     * 单次更新（非阻塞）
     */
    void update();

    // ============================================================
    // 状态查询
    // ============================================================

    State getState() const { return state_; }
    PathMode getPathMode() const { return pathMode_; }
    void setPathMode(PathMode mode) { pathMode_ = mode; }

    const Pose& getCurrentPose() const { return currentPose_; }
    const Point3D& getCurrentGoal() const { return currentGoal_; }

    // 统计信息
    struct Statistics {
        double exploredVolume = 0.0;        // 已探索体积
        int waypointsVisited = 0;           // 已访问路点数
        int frontiersDetected = 0;          // 检测到的前沿数
        int pathsPlanned = 0;               // 规划的路径数
        int unreachableGoals = 0;           // 不可达目标数
        double totalDistance = 0.0;         // 总飞行距离
        std::chrono::steady_clock::time_point startTime;
        double elapsedSeconds = 0.0;
    };

    Statistics getStatistics() const { return stats_; }

    // ============================================================
    // 模块访问
    // ============================================================

    OctoMapManager& getOctoMapManager() { return *octomapManager_; }
    const OctoMapManager& getOctoMapManager() const { return *octomapManager_; }

    FrontierDetector& getFrontierDetector() { return *frontierDetector_; }
    PathPlanner& getPathPlanner() { return *pathPlanner_; }

private:
    ExplorationConfig config_;
    State state_{State::IDLE};
    PathMode pathMode_{PathMode::ASTAR};

    // 核心模块
    std::unique_ptr<OctoMapManager> octomapManager_;
    std::unique_ptr<FrontierDetector> frontierDetector_;
    std::unique_ptr<PathPlanner> pathPlanner_;
    std::unique_ptr<MqttClient> mqttClient_;

    // 当前状态
    Pose currentPose_;
    Point3D currentGoal_;
    Path3D currentPath_;
    bool hasValidGoal_{false};
    std::string currentMissionId_;

    // 统计
    Statistics stats_;

    // 控制
    std::atomic<bool> running_{false};
    std::atomic<bool> paused_{false};

    // 时间控制
    std::chrono::steady_clock::time_point lastUpdateTime_;
    std::chrono::steady_clock::time_point lastGoalTime_;

    // ============================================================
    // 内部方法
    // ============================================================

    /**
     * 处理新的点云数据
     */
    void onPointCloud(const PointCloud& cloud);

    /**
     * 处理位姿更新
     */
    void onOdometry(const Pose& pose);

    /**
     * 选择下一个探索目标
     */
    bool selectNextGoal();

    /**
     * 规划到目标的路径
     */
    bool planPath(const Point3D& goal);

    /**
     * 发布任务到无人机
     */
    bool publishMission(const Path3D& path);

    /**
     * 检查是否到达当前目标
     */
    bool hasReachedGoal() const;

    /**
     * 检查探索是否完成
     */
    bool isExplorationComplete() const;

    /**
     * 处理目标不可达
     */
    void handleUnreachableGoal();

    /**
     * 生成任务ID
     */
    std::string generateMissionId();

    /**
     * 更新统计信息
     */
    void updateStatistics();

    /**
     * 打印状态信息
     */
    void printStatus() const;
};

}  // namespace exploration
