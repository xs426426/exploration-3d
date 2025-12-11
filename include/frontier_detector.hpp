#pragma once

#include "types.hpp"
#include "octomap_manager.hpp"
#include <vector>
#include <memory>
#include <unordered_set>

namespace exploration {

/**
 * 3D 前沿点检测器
 * 基于 OctoMap 检测未探索边界
 */
class FrontierDetector {
public:
    explicit FrontierDetector(const ExplorationConfig& config);

    /**
     * 检测前沿点簇
     * @param octomapManager OctoMap管理器
     * @param currentPos 当前无人机位置
     * @return 前沿点簇列表（按评分排序）
     */
    std::vector<FrontierCluster> detectFrontiers(
        const OctoMapManager& octomapManager,
        const Point3D& currentPos);

    /**
     * 选择最优前沿点
     * @param frontiers 前沿点簇列表
     * @param currentPos 当前位置
     * @param lastDirection 上一个目标方向（用于一致性奖励）
     * @return 最优前沿点，如果没有返回空簇
     */
    FrontierCluster selectBestFrontier(
        const std::vector<FrontierCluster>& frontiers,
        const Point3D& currentPos,
        const Point3D* lastDirection = nullptr);

    /**
     * 设置已访问目标（用于历史惩罚）
     */
    void addVisitedGoal(const Point3D& goal);
    void clearVisitedGoals();

    /**
     * 设置不可达目标黑名单
     */
    void addUnreachableGoal(const Point3D& goal);
    void clearUnreachableGoals();

    /**
     * 记录无人机轨迹点（用于避免路径重复）
     */
    void recordTrajectoryPoint(const Point3D& point);

    /**
     * 清除轨迹历史
     */
    void clearTrajectoryHistory();

    /**
     * 设置环境边界（用于覆盖率计算）
     */
    void setEnvironmentBounds(const Point3D& minBound, const Point3D& maxBound);

    /**
     * 更新覆盖率
     */
    void updateCoverage(const OctoMapManager& octomapManager);

    /**
     * 获取探索覆盖率
     */
    double getExplorationCoverage() const { return explorationCoverage_; }

    /**
     * 检查是否在黑名单中
     */
    bool isUnreachable(const Point3D& point, double threshold = 2.0) const;

private:
    ExplorationConfig config_;

    // 历史记录
    std::vector<Point3D> visitedGoals_;
    std::vector<Point3D> unreachableGoals_;
    std::vector<Point3D> trajectoryHistory_;           // 轨迹历史
    std::unordered_set<int64_t> visitedGridCells_;     // 已访问的网格单元（快速查询）

    // 环境边界
    Point3D envMinBound_, envMaxBound_;
    bool boundsSet_ = false;

    // 覆盖率
    double explorationCoverage_ = 0.0;

    // 轨迹记录间隔（米）
    static constexpr double trajectoryGridSize_ = 0.5;

    /**
     * 对前沿点进行聚类
     */
    std::vector<FrontierCluster> clusterFrontiers(
        const std::vector<Point3D>& frontierCells,
        double clusterRadius,
        int minClusterSize);

    /**
     * 计算前沿点簇评分
     */
    double calculateScore(
        const FrontierCluster& cluster,
        const Point3D& currentPos,
        const Point3D* lastDirection);

    /**
     * 计算历史惩罚
     */
    double calculateHistoryPenalty(const Point3D& point) const;

    /**
     * 计算轨迹重复惩罚（避免重复走过的路径）
     */
    double calculateTrajectoryPenalty(const Point3D& point) const;

    /**
     * 将3D坐标转换为网格键值（用于去重）
     */
    int64_t pointToGridKey(const Point3D& point, double gridSize) const;
};

}  // namespace exploration
