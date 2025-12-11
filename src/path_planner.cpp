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

    // 首先检查直线路径（只检查已知障碍物，允许穿越未知）
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

    // 检查起点 - 只检查已知占据（无人机已经在那里了）
    if (octomapManager.isOccupied(start, config_.safetyMargin)) {
        std::cerr << "[PathPlanner] 起点在障碍物内" << std::endl;
        return result;
    }
    // 检查终点 - 也只检查已知占据（允许飞向未知区域探索）
    if (octomapManager.isOccupied(goal, config_.safetyMargin)) {
        std::cerr << "[PathPlanner] 终点在障碍物内" << std::endl;
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
    // 使用 isPathClearIgnoreUnknown 允许穿越未知空间进行探索
    return octomapManager.isPathClearIgnoreUnknown(start, goal,
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
    // 只检查已知障碍物，允许穿越未知空间进行探索
    return !octomapManager.isOccupied(p, config_.safetyMargin);
}

Point3D PathPlanner::findObservationPoint(const Point3D& frontier, const Point3D& currentPos,
                                           const OctoMapManager& octomapManager) const {
    double resolution = octomapManager.getResolution();

    // 从当前位置向前沿点方向的基础向量
    Eigen::Vector3d baseDir = frontier.toEigen() - currentPos.toEigen();
    double totalDist = baseDir.norm();

    if (totalDist < 0.01) {
        return Point3D(0, 0, 0);  // 无效
    }

    baseDir.normalize();

    // ============================================================
    // 多点采样策略：围绕前沿点在不同角度和距离上采样
    // ============================================================

    struct CandidatePoint {
        Point3D point;
        double score;      // 评分：距离+可见性
        double distToFrontier;
    };

    std::vector<CandidatePoint> candidates;

    // 采样参数
    double minObserveDist = 1.0;   // 最小观察距离（距离前沿点）
    double maxObserveDist = 3.0;   // 最大观察距离
    double distStep = 0.5;         // 距离步长
    int numAngles = 8;             // 水平方向采样角度数
    double angleStep = 2.0 * M_PI / numAngles;

    // 计算两个垂直于baseDir的基向量，用于生成采样圆
    Eigen::Vector3d up(0, 0, 1);
    Eigen::Vector3d perpX = baseDir.cross(up);
    if (perpX.norm() < 0.01) {
        perpX = Eigen::Vector3d(1, 0, 0);
    }
    perpX.normalize();
    Eigen::Vector3d perpY = baseDir.cross(perpX);
    perpY.normalize();

    // 方案1：沿着当前位置到前沿点的直线方向采样
    for (double d = 0.5; d <= totalDist * 0.8; d += distStep) {
        Point3D candidate(
            currentPos.x + baseDir.x() * d,
            currentPos.y + baseDir.y() * d,
            currentPos.z + baseDir.z() * d
        );

        // 检查候选点是否安全（无已知障碍物）
        if (!octomapManager.isOccupied(candidate, config_.safetyMargin)) {
            double distToFrontier = candidate.distanceTo(frontier);
            double distToCurrent = candidate.distanceTo(currentPos);

            // 评分：优先选择离前沿点适中距离、离当前位置近的点
            double score = 10.0;  // 基础分
            score -= 0.5 * distToCurrent;  // 距离当前位置越近越好
            score -= std::abs(distToFrontier - 2.0) * 0.3;  // 距离前沿点2米左右最好

            candidates.push_back({candidate, score, distToFrontier});
        }
    }

    // 方案2：围绕前沿点采样（多角度）
    for (double dist = minObserveDist; dist <= maxObserveDist; dist += distStep) {
        for (int i = 0; i < numAngles; ++i) {
            double angle = i * angleStep;

            // 从前沿点向外偏移
            Eigen::Vector3d offset = -baseDir * dist;  // 先沿反方向
            // 加上一些侧向偏移
            offset += perpX * (dist * 0.3 * std::cos(angle));
            offset += perpY * (dist * 0.3 * std::sin(angle));

            Point3D candidate(
                frontier.x + offset.x(),
                frontier.y + offset.y(),
                frontier.z + offset.z()
            );

            // 高度限制
            if (candidate.z < config_.minHeight || candidate.z > config_.maxHeight) {
                continue;
            }

            // 检查候选点是否安全
            if (!octomapManager.isOccupied(candidate, config_.safetyMargin)) {
                double distToCurrent = candidate.distanceTo(currentPos);
                double distToFrontier = candidate.distanceTo(frontier);

                // 检查从候选点到前沿点的视线是否清晰
                bool hasGoodView = octomapManager.isPathClearIgnoreUnknown(
                    candidate, frontier, resolution, 0.0);

                double score = 5.0;  // 基础分（比直线方案低）
                score -= 0.3 * distToCurrent;  // 距离当前位置
                if (hasGoodView) {
                    score += 3.0;  // 有良好视野加分
                }

                candidates.push_back({candidate, score, distToFrontier});
            }
        }
    }

    // 方案3：在当前位置附近采样（如果上述方案都失败）
    for (double dx = -1.0; dx <= 1.0; dx += 0.5) {
        for (double dy = -1.0; dy <= 1.0; dy += 0.5) {
            if (std::abs(dx) < 0.1 && std::abs(dy) < 0.1) continue;

            Point3D candidate(
                currentPos.x + dx,
                currentPos.y + dy,
                currentPos.z
            );

            if (!octomapManager.isOccupied(candidate, config_.safetyMargin)) {
                double distToFrontier = candidate.distanceTo(frontier);

                // 检查是否比当前位置更靠近前沿点
                if (distToFrontier < totalDist - 0.3) {
                    double score = 2.0;  // 低优先级
                    score -= distToFrontier * 0.1;

                    candidates.push_back({candidate, score, distToFrontier});
                }
            }
        }
    }

    // 选择最佳候选点
    if (!candidates.empty()) {
        // 按评分排序
        std::sort(candidates.begin(), candidates.end(),
                  [](const CandidatePoint& a, const CandidatePoint& b) {
                      return a.score > b.score;
                  });

        const auto& best = candidates[0];

        // 确保候选点距离当前位置足够远（避免微小移动）
        if (best.point.distanceTo(currentPos) > 0.3) {
            std::cout << "[PathPlanner] 找到观察点: (" << best.point.x << ", "
                      << best.point.y << ", " << best.point.z
                      << ") 距当前位置 " << currentPos.distanceTo(best.point)
                      << "m, 距前沿点 " << best.distToFrontier
                      << "m, 评分=" << best.score
                      << " (共" << candidates.size() << "个候选点)" << std::endl;
            return best.point;
        }
    }

    // 如果所有候选点都太近，返回当前位置（原地观察）
    if (!octomapManager.isOccupied(currentPos, config_.safetyMargin)) {
        std::cout << "[PathPlanner] 无有效候选点，使用当前位置作为观察点（原地观察）" << std::endl;
        return currentPos;
    }

    std::cerr << "[PathPlanner] 无法为前沿点找到观察点" << std::endl;
    return Point3D(0, 0, 0);  // 无效
}

}  // namespace exploration
