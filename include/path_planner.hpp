#pragma once

#include "types.hpp"
#include "octomap_manager.hpp"
#include <vector>
#include <queue>
#include <unordered_map>
#include <memory>

namespace exploration {

/**
 * 3D A* 路径规划器
 */
class PathPlanner {
public:
    explicit PathPlanner(const ExplorationConfig& config);

    /**
     * 3D A* 路径搜索
     * @param start 起点
     * @param goal 终点
     * @param octomapManager OctoMap管理器
     * @return 路径（空表示无法到达）
     */
    Path3D planPath(const Point3D& start, const Point3D& goal,
                    const OctoMapManager& octomapManager);

    /**
     * 为前沿点找到最近的可达观察点
     * 前沿点在未知空间边界，需要找一个在已知空闲空间的观察点
     * @param frontier 前沿点位置
     * @param currentPos 当前位置
     * @param octomapManager OctoMap管理器
     * @return 观察点（无效则返回原点）
     */
    Point3D findObservationPoint(const Point3D& frontier, const Point3D& currentPos,
                                  const OctoMapManager& octomapManager) const;

    /**
     * 检查直线路径是否可通行
     */
    bool isDirectPathClear(const Point3D& start, const Point3D& goal,
                           const OctoMapManager& octomapManager) const;

    /**
     * 简化路径（移除共线点）
     */
    Path3D simplifyPath(const Path3D& path, const OctoMapManager& octomapManager) const;

private:
    ExplorationConfig config_;

    // A* 节点
    struct AStarNode {
        int x, y, z;     // 栅格坐标
        double g = 0;    // 起点到当前的实际代价
        double f = 0;    // 估计总代价 f = g + h

        bool operator>(const AStarNode& other) const {
            return f > other.f;
        }
    };

    // 坐标哈希
    struct CoordHash {
        size_t operator()(const std::tuple<int, int, int>& coord) const {
            auto h1 = std::hash<int>{}(std::get<0>(coord));
            auto h2 = std::hash<int>{}(std::get<1>(coord));
            auto h3 = std::hash<int>{}(std::get<2>(coord));
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };

    using CoordKey = std::tuple<int, int, int>;

    /**
     * 世界坐标转栅格坐标
     */
    CoordKey worldToGrid(const Point3D& point, double resolution) const;

    /**
     * 栅格坐标转世界坐标
     */
    Point3D gridToWorld(const CoordKey& coord, double resolution) const;

    /**
     * 启发式函数（3D欧几里得距离）
     */
    double heuristic(const CoordKey& a, const CoordKey& b, double resolution) const;

    /**
     * 检查栅格是否可通行
     */
    bool isTraversable(const CoordKey& coord, const OctoMapManager& octomapManager,
                       double resolution) const;

    /**
     * 26邻域
     */
    static const int neighbors_[26][3];
    static const double neighborCosts_[26];
};

}  // namespace exploration
