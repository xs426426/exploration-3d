#include "path_planner.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

namespace exploration {

// 26邻域偏移
const int PathPlanner::neighbors_[26][3] = {
    // 6个面邻居 (cost = 1.0)
    {-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1},
    // 12个边邻居 (cost = sqrt(2) ≈ 1.414)
    {-1, -1, 0}, {-1, 1, 0}, {1, -1, 0}, {1, 1, 0},
    {-1, 0, -1}, {-1, 0, 1}, {1, 0, -1}, {1, 0, 1},
    {0, -1, -1}, {0, -1, 1}, {0, 1, -1}, {0, 1, 1},
    // 8个角邻居 (cost = sqrt(3) ≈ 1.732)
    {-1, -1, -1}, {-1, -1, 1}, {-1, 1, -1}, {-1, 1, 1},
    {1, -1, -1}, {1, -1, 1}, {1, 1, -1}, {1, 1, 1}
};

const double PathPlanner::neighborCosts_[26] = {
    // 6个面邻居
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    // 12个边邻居
    1.414, 1.414, 1.414, 1.414, 1.414, 1.414, 1.414, 1.414, 1.414, 1.414, 1.414, 1.414,
    // 8个角邻居
    1.732, 1.732, 1.732, 1.732, 1.732, 1.732, 1.732, 1.732
};

PathPlanner::PathPlanner(const ExplorationConfig& config)
    : config_(config)
{
}

Path3D PathPlanner::planPath(const Point3D& start, const Point3D& goal,
                              const OctoMapManager& octomapManager) {
    Path3D result;
    result.isValid = false;

    double resolution = octomapManager.getResolution();

    // 首先检查直线路径
    if (isDirectPathClear(start, goal, octomapManager)) {
        result.waypoints.push_back(start);
        result.waypoints.push_back(goal);
        result.isValid = true;
        result.calculateLength();
        std::cout << "[PathPlanner] 直线路径可达，长度=" << result.totalLength << "m" << std::endl;
        return result;
    }

    // A* 搜索
    CoordKey startKey = worldToGrid(start, resolution);
    CoordKey goalKey = worldToGrid(goal, resolution);

    // 检查起点 - 只检查已知占据（无人机已经在那里了，未知没关系）
    if (octomapManager.isOccupied(start, config_.safetyMargin)) {
        std::cerr << "[PathPlanner] 起点在障碍物内" << std::endl;
        return result;
    }
    // 检查终点 - 需要检查未知（不能飞到未知区域）
    if (!isTraversable(goalKey, octomapManager, resolution)) {
        std::cerr << "[PathPlanner] 终点不可通行" << std::endl;
        return result;
    }

    // 优先队列（最小堆）
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> openSet;

    // 访问记录
    std::unordered_map<CoordKey, double, CoordHash> gScore;
    std::unordered_map<CoordKey, CoordKey, CoordHash> cameFrom;

    // 初始化
    AStarNode startNode;
    startNode.x = std::get<0>(startKey);
    startNode.y = std::get<1>(startKey);
    startNode.z = std::get<2>(startKey);
    startNode.g = 0;
    startNode.f = heuristic(startKey, goalKey, resolution);

    openSet.push(startNode);
    gScore[startKey] = 0;

    int iterations = 0;
    const int maxIterations = config_.astarMaxIterations;

    while (!openSet.empty() && iterations < maxIterations) {
        iterations++;

        AStarNode current = openSet.top();
        openSet.pop();

        CoordKey currentKey = {current.x, current.y, current.z};

        // 到达目标
        if (currentKey == goalKey) {
            // 重建路径
            std::vector<Point3D> path;
            CoordKey key = goalKey;

            while (cameFrom.find(key) != cameFrom.end()) {
                path.push_back(gridToWorld(key, resolution));
                key = cameFrom[key];
            }
            path.push_back(gridToWorld(startKey, resolution));

            std::reverse(path.begin(), path.end());

            result.waypoints = path;
            result.isValid = true;
            result.calculateLength();

            std::cout << "[PathPlanner] A*路径找到，迭代=" << iterations
                      << ", 路点=" << path.size()
                      << ", 长度=" << result.totalLength << "m" << std::endl;

            // 简化路径
            return simplifyPath(result, octomapManager);
        }

        // 已经处理过更优路径
        if (gScore.find(currentKey) != gScore.end() &&
            current.g > gScore[currentKey]) {
            continue;
        }

        // 遍历邻居
        for (int i = 0; i < 26; ++i) {
            CoordKey neighborKey = {
                current.x + neighbors_[i][0],
                current.y + neighbors_[i][1],
                current.z + neighbors_[i][2]
            };

            // 检查可通行性
            if (!isTraversable(neighborKey, octomapManager, resolution)) continue;

            // 对角线移动检查（防止穿墙）
            if (neighbors_[i][0] != 0 && neighbors_[i][1] != 0) {
                CoordKey adj1 = {current.x + neighbors_[i][0], current.y, current.z};
                CoordKey adj2 = {current.x, current.y + neighbors_[i][1], current.z};
                if (!isTraversable(adj1, octomapManager, resolution) ||
                    !isTraversable(adj2, octomapManager, resolution)) continue;
            }
            if (neighbors_[i][0] != 0 && neighbors_[i][2] != 0) {
                CoordKey adj1 = {current.x + neighbors_[i][0], current.y, current.z};
                CoordKey adj2 = {current.x, current.y, current.z + neighbors_[i][2]};
                if (!isTraversable(adj1, octomapManager, resolution) ||
                    !isTraversable(adj2, octomapManager, resolution)) continue;
            }
            if (neighbors_[i][1] != 0 && neighbors_[i][2] != 0) {
                CoordKey adj1 = {current.x, current.y + neighbors_[i][1], current.z};
                CoordKey adj2 = {current.x, current.y, current.z + neighbors_[i][2]};
                if (!isTraversable(adj1, octomapManager, resolution) ||
                    !isTraversable(adj2, octomapManager, resolution)) continue;
            }

            double tentativeG = current.g + neighborCosts_[i] * resolution;

            if (gScore.find(neighborKey) == gScore.end() ||
                tentativeG < gScore[neighborKey]) {

                gScore[neighborKey] = tentativeG;
                cameFrom[neighborKey] = currentKey;

                AStarNode neighborNode;
                neighborNode.x = std::get<0>(neighborKey);
                neighborNode.y = std::get<1>(neighborKey);
                neighborNode.z = std::get<2>(neighborKey);
                neighborNode.g = tentativeG;
                neighborNode.f = tentativeG + heuristic(neighborKey, goalKey, resolution);

                openSet.push(neighborNode);
            }
        }
    }

    std::cerr << "[PathPlanner] A*搜索失败，迭代=" << iterations << std::endl;
    return result;
}

bool PathPlanner::isDirectPathClear(const Point3D& start, const Point3D& goal,
                                     const OctoMapManager& octomapManager) const {
    return octomapManager.isPathClear(start, goal,
                                       octomapManager.getResolution(),
                                       config_.safetyMargin);
}

Path3D PathPlanner::simplifyPath(const Path3D& path,
                                  const OctoMapManager& octomapManager) const {
    if (path.waypoints.size() <= 2) return path;

    Path3D simplified;
    simplified.isValid = path.isValid;
    simplified.waypoints.push_back(path.waypoints.front());

    double tolerance = config_.astarSimplifyTolerance;

    for (size_t i = 1; i < path.waypoints.size() - 1; ++i) {
        const Point3D& prev = simplified.waypoints.back();
        const Point3D& curr = path.waypoints[i];
        const Point3D& next = path.waypoints[i + 1];

        // 计算点到直线的距离
        Eigen::Vector3d a = curr.toEigen() - prev.toEigen();
        Eigen::Vector3d b = next.toEigen() - prev.toEigen();

        double bLen = b.norm();
        if (bLen < 0.001) {
            simplified.waypoints.push_back(curr);
            continue;
        }

        Eigen::Vector3d bNorm = b / bLen;
        double proj = a.dot(bNorm);
        Eigen::Vector3d perpendicular = a - proj * bNorm;
        double dist = perpendicular.norm();

        if (dist > tolerance) {
            simplified.waypoints.push_back(curr);
        }
    }

    simplified.waypoints.push_back(path.waypoints.back());
    simplified.calculateLength();

    std::cout << "[PathPlanner] 路径简化: " << path.waypoints.size()
              << " -> " << simplified.waypoints.size() << " 个路点" << std::endl;

    return simplified;
}

PathPlanner::CoordKey PathPlanner::worldToGrid(const Point3D& point, double resolution) const {
    return {
        static_cast<int>(std::floor(point.x / resolution)),
        static_cast<int>(std::floor(point.y / resolution)),
        static_cast<int>(std::floor(point.z / resolution))
    };
}

Point3D PathPlanner::gridToWorld(const CoordKey& coord, double resolution) const {
    return Point3D(
        (std::get<0>(coord) + 0.5) * resolution,
        (std::get<1>(coord) + 0.5) * resolution,
        (std::get<2>(coord) + 0.5) * resolution
    );
}

double PathPlanner::heuristic(const CoordKey& a, const CoordKey& b, double resolution) const {
    double dx = std::get<0>(a) - std::get<0>(b);
    double dy = std::get<1>(a) - std::get<1>(b);
    double dz = std::get<2>(a) - std::get<2>(b);
    return std::sqrt(dx*dx + dy*dy + dz*dz) * resolution;
}

bool PathPlanner::isTraversable(const CoordKey& coord, const OctoMapManager& octomapManager,
                                 double resolution) const {
    Point3D p = gridToWorld(coord, resolution);
    return !octomapManager.isInCollision(p, config_.safetyMargin);
}

Point3D PathPlanner::findObservationPoint(const Point3D& frontier, const Point3D& currentPos,
                                           const OctoMapManager& octomapManager) const {
    double resolution = octomapManager.getResolution();

    // 从前沿点向当前位置方向搜索，找到第一个已知空闲点
    Eigen::Vector3d dir = currentPos.toEigen() - frontier.toEigen();
    double dist = dir.norm();

    if (dist < 0.01) {
        return Point3D(0, 0, 0);  // 无效
    }

    dir.normalize();

    // 沿着方向搜索，步长为分辨率
    double searchStep = resolution;
    double maxSearchDist = 3.0;  // 最多搜索3米

    for (double d = searchStep; d <= maxSearchDist; d += searchStep) {
        Point3D candidate(
            frontier.x + dir.x() * d,
            frontier.y + dir.y() * d,
            frontier.z + dir.z() * d
        );

        // 检查该点是否在已知空闲空间（不在碰撞中）
        if (!octomapManager.isInCollision(candidate, config_.safetyMargin)) {
            std::cout << "[PathPlanner] 找到观察点: (" << candidate.x << ", "
                      << candidate.y << ", " << candidate.z
                      << ") 距前沿 " << d << "m" << std::endl;
            return candidate;
        }
    }

    // 如果直线方向没找到，尝试球面搜索
    double searchRadius = 1.0;  // 搜索半径
    for (double r = searchStep; r <= searchRadius; r += searchStep) {
        // 在球面上采样
        for (double theta = 0; theta < M_PI; theta += M_PI / 6) {
            for (double phi = 0; phi < 2 * M_PI; phi += M_PI / 6) {
                Point3D candidate(
                    frontier.x + r * std::sin(theta) * std::cos(phi),
                    frontier.y + r * std::sin(theta) * std::sin(phi),
                    frontier.z + r * std::cos(theta)
                );

                if (!octomapManager.isInCollision(candidate, config_.safetyMargin)) {
                    std::cout << "[PathPlanner] 球面搜索找到观察点: (" << candidate.x << ", "
                              << candidate.y << ", " << candidate.z << ")" << std::endl;
                    return candidate;
                }
            }
        }
    }

    std::cerr << "[PathPlanner] 无法为前沿点找到观察点" << std::endl;
    return Point3D(0, 0, 0);  // 无效
}

}  // namespace exploration
