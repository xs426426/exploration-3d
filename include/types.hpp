#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <Eigen/Core>

namespace exploration {

// ============================================================
// 基础类型定义
// ============================================================

/// 3D点
struct Point3D {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Point3D() = default;
    Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Eigen::Vector3d toEigen() const {
        return Eigen::Vector3d(x, y, z);
    }

    static Point3D fromEigen(const Eigen::Vector3d& v) {
        return Point3D(v.x(), v.y(), v.z());
    }

    double distanceTo(const Point3D& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        double dz = z - other.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    double distanceXY(const Point3D& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx*dx + dy*dy);
    }
};

/// 位姿 (位置 + 姿态四元数)
struct Pose {
    Point3D position;
    double qw = 1.0;  // 四元数
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double yaw = 0.0; // 欧拉角 yaw (弧度)
};

/// 点云
struct PointCloud {
    std::vector<Point3D> points;
    uint64_t timestamp = 0;  // 毫秒时间戳
};

/// 前沿点簇
struct FrontierCluster {
    Point3D centroid;           // 簇中心
    std::vector<Point3D> cells; // 前沿格子
    double size = 0.0;          // 簇大小（格子数）
    double infoGain = 0.0;      // 信息增益估计
    double score = 0.0;         // 综合评分
};

/// 3D路径
struct Path3D {
    std::vector<Point3D> waypoints;
    double totalLength = 0.0;
    bool isValid = false;

    void calculateLength() {
        totalLength = 0.0;
        for (size_t i = 1; i < waypoints.size(); ++i) {
            totalLength += waypoints[i].distanceTo(waypoints[i-1]);
        }
    }
};

/// 任务定义
struct Mission {
    std::string id;
    std::vector<Point3D> waypoints;
    double yaw = 0.0;
};

// ============================================================
// 探索状态
// ============================================================

enum class ExplorationState {
    IDLE,           // 空闲
    EXPLORING,      // 探索中
    PAUSED,         // 暂停
    RETURNING_HOME, // 返航中
    COMPLETED,      // 完成
    ERROR           // 错误
};

inline std::string stateToString(ExplorationState state) {
    switch (state) {
        case ExplorationState::IDLE: return "IDLE";
        case ExplorationState::EXPLORING: return "EXPLORING";
        case ExplorationState::PAUSED: return "PAUSED";
        case ExplorationState::RETURNING_HOME: return "RETURNING_HOME";
        case ExplorationState::COMPLETED: return "COMPLETED";
        case ExplorationState::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

// ============================================================
// 配置结构
// ============================================================

struct ExplorationConfig {
    // 地图参数
    double resolution = 0.1;         // 体素分辨率 (米)
    double mapSizeX = 20.0;          // 地图X范围 (米)
    double mapSizeY = 20.0;          // 地图Y范围 (米)
    double mapSizeZ = 4.0;           // 地图Z范围 (米)

    // 探索参数
    double maxExplorationDistance = 20.0;  // 最大探索距离 (米)
    double maxExplorationTime = 600.0;     // 最大探索时间 (秒)
    double minHeight = 0.5;                // 最小飞行高度 (米)
    double maxHeight = 3.0;                // 最大飞行高度 (米)
    double defaultHeight = 1.0;            // 默认探索高度 (米)

    // 前沿点检测参数
    double clusterRadius = 1.0;      // 聚类半径 (米)
    int minClusterSize = 5;          // 最小簇大小

    // 评分权重
    double weightInfoGain = 0.5;     // 信息增益权重
    double weightDistance = 0.3;     // 距离权重
    double weightConsistency = 0.2;  // 方向一致性权重

    // A* 参数
    int astarMaxIterations = 50000;      // A*最大迭代次数
    double astarHeuristicWeight = 1.0;   // 启发式权重
    double astarSimplifyTolerance = 0.2; // 路径简化容差 (米)

    // 探索引擎参数
    double goalTolerance = 0.5;          // 目标到达容差 (米)
    double goalTimeout = 60.0;           // 目标超时时间 (秒)
    double updateRate = 10.0;            // 更新频率 (Hz)
    std::string pathPlanningMode = "astar";  // 路径规划模式

    // MQTT 参数
    std::string mqttBroker = "tcp://localhost:1883";
    std::string mqttClientId = "exploration_3d";
    std::string mqttTopicPointCloud = "/daf/pointcloud";
    std::string mqttTopicOdometry = "/daf/odometry";
    std::string mqttTopicMission = "/daf/mission";
    std::string mqttTopicExecution = "/daf/execution";

    // 安全参数
    double safetyMargin = 0.3;       // 安全边距 (米)
    double arrivalThreshold = 0.5;   // 到达判定阈值 (米)
    double stuckVelocityThreshold = 0.1;  // 僵死判定速度阈值 (m/s)
    double stuckTimeThreshold = 3.0;      // 僵死判定时间 (秒)
};

// ============================================================
// 回调类型
// ============================================================

using PointCloudCallback = std::function<void(const PointCloud&)>;
using OdometryCallback = std::function<void(const Pose&)>;
using GoalReachedCallback = std::function<void(const Point3D&)>;

}  // namespace exploration
