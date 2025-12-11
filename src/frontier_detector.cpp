#include "frontier_detector.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_set>

namespace exploration {

FrontierDetector::FrontierDetector(const ExplorationConfig& config)
    : config_(config)
{
}

std::vector<FrontierCluster> FrontierDetector::detectFrontiers(
    const OctoMapManager& octomapManager,
    const Point3D& currentPos)
{
    // 1. 获取所有前沿体素
    std::vector<Point3D> frontierCells = octomapManager.detectFrontierCells();

    if (frontierCells.empty()) {
        return {};
    }

    // 2. 过滤距离太远的前沿点
    std::vector<Point3D> filteredCells;
    for (const auto& cell : frontierCells) {
        double dist = cell.distanceTo(currentPos);
        if (dist <= config_.maxExplorationDistance) {
            // 高度限制
            if (cell.z >= config_.minHeight && cell.z <= config_.maxHeight) {
                filteredCells.push_back(cell);
            }
        }
    }

    if (filteredCells.empty()) {
        return {};
    }

    // 3. 聚类
    std::vector<FrontierCluster> clusters = clusterFrontiers(
        filteredCells,
        config_.clusterRadius,
        config_.minClusterSize);

    // 4. 过滤已访问和不可达的前沿簇
    std::vector<FrontierCluster> validClusters;
    for (auto& cluster : clusters) {
        // 检查是否在不可达列表中
        if (isUnreachable(cluster.centroid, 0.5)) {
            continue;
        }

        // 检查是否在已访问列表中（使用较小的阈值）
        bool visited = false;
        for (const auto& visitedGoal : visitedGoals_) {
            if (cluster.centroid.distanceTo(visitedGoal) < 0.5) {
                visited = true;
                break;
            }
        }
        if (visited) {
            continue;
        }

        validClusters.push_back(cluster);
    }

    // 5. 计算评分并排序
    for (auto& cluster : validClusters) {
        cluster.score = calculateScore(cluster, currentPos, nullptr);
        // 应用历史惩罚
        cluster.score -= config_.weightDistance * calculateHistoryPenalty(cluster.centroid);
        // 应用轨迹重复惩罚（新增）
        cluster.score -= 0.5 * calculateTrajectoryPenalty(cluster.centroid);
    }

    std::sort(validClusters.begin(), validClusters.end(),
              [](const FrontierCluster& a, const FrontierCluster& b) {
                  return a.score > b.score;
              });

    std::cout << "[FrontierDetector] 检测到 " << validClusters.size()
              << " 个前沿簇 (原始体素: " << frontierCells.size()
              << ", 过滤后: " << filteredCells.size()
              << ", 排除已访问/不可达: " << (clusters.size() - validClusters.size()) << ")";

    if (boundsSet_) {
        std::cout << " 覆盖率: " << (explorationCoverage_ * 100.0) << "%";
    }
    std::cout << std::endl;

    return validClusters;
}

std::vector<FrontierCluster> FrontierDetector::clusterFrontiers(
    const std::vector<Point3D>& frontierCells,
    double clusterRadius,
    int minClusterSize)
{
    std::vector<FrontierCluster> clusters;
    std::vector<bool> visited(frontierCells.size(), false);

    for (size_t i = 0; i < frontierCells.size(); ++i) {
        if (visited[i]) continue;

        FrontierCluster cluster;
        cluster.cells.push_back(frontierCells[i]);
        visited[i] = true;

        // 简单贪婪聚类
        for (size_t j = i + 1; j < frontierCells.size(); ++j) {
            if (visited[j]) continue;

            // 检查是否与簇内任意点距离小于阈值
            bool inCluster = false;
            for (const auto& cell : cluster.cells) {
                if (cell.distanceTo(frontierCells[j]) < clusterRadius) {
                    inCluster = true;
                    break;
                }
            }

            if (inCluster) {
                cluster.cells.push_back(frontierCells[j]);
                visited[j] = true;
            }
        }

        // 过滤太小的簇
        if (static_cast<int>(cluster.cells.size()) >= minClusterSize) {
            // 计算簇中心
            double sumX = 0, sumY = 0, sumZ = 0;
            for (const auto& cell : cluster.cells) {
                sumX += cell.x;
                sumY += cell.y;
                sumZ += cell.z;
            }
            cluster.centroid.x = sumX / cluster.cells.size();
            cluster.centroid.y = sumY / cluster.cells.size();
            cluster.centroid.z = sumZ / cluster.cells.size();
            cluster.size = static_cast<double>(cluster.cells.size());

            // 信息增益估计（基于簇大小）
            cluster.infoGain = std::min(cluster.size / 50.0, 1.0);

            clusters.push_back(cluster);
        }
    }

    return clusters;
}

FrontierCluster FrontierDetector::selectBestFrontier(
    const std::vector<FrontierCluster>& frontiers,
    const Point3D& currentPos,
    const Point3D* lastDirection)
{
    FrontierCluster best;
    double bestScore = -std::numeric_limits<double>::infinity();

    for (const auto& cluster : frontiers) {
        // 检查是否在黑名单中
        if (isUnreachable(cluster.centroid)) {
            continue;
        }

        double score = calculateScore(cluster, currentPos, lastDirection);

        // 历史惩罚
        score -= config_.weightDistance * calculateHistoryPenalty(cluster.centroid);

        if (score > bestScore) {
            bestScore = score;
            best = cluster;
            best.score = score;
        }
    }

    if (bestScore > -std::numeric_limits<double>::infinity()) {
        std::cout << "[FrontierDetector] 选择最优前沿: ("
                  << best.centroid.x << ", " << best.centroid.y << ", " << best.centroid.z
                  << ") 评分=" << best.score << std::endl;
    }

    return best;
}

double FrontierDetector::calculateScore(
    const FrontierCluster& cluster,
    const Point3D& currentPos,
    const Point3D* lastDirection)
{
    double score = 0.0;

    // 0. 距离过滤：太近的前沿点直接给低分
    double distance = cluster.centroid.distanceTo(currentPos);
    if (distance < 0.5) {  // 小于0.5米的前沿点忽略
        return -1000.0;
    }

    // 1. 信息增益
    score += config_.weightInfoGain * cluster.infoGain;

    // 2. 距离成本（适中距离最好，太近太远都不好）
    // 最佳距离在 2-5 米
    double optimalDist = 3.0;
    double distScore = 1.0 - std::abs(distance - optimalDist) / optimalDist;
    distScore = std::max(0.0, distScore);
    score += config_.weightDistance * distScore;

    // 3. 方向一致性（如果有上一个方向）
    if (lastDirection != nullptr) {
        double dx = cluster.centroid.x - currentPos.x;
        double dy = cluster.centroid.y - currentPos.y;
        double dz = cluster.centroid.z - currentPos.z;
        double norm = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (norm > 0.01) {
            dx /= norm;
            dy /= norm;
            dz /= norm;

            double lastNorm = std::sqrt(
                lastDirection->x * lastDirection->x +
                lastDirection->y * lastDirection->y +
                lastDirection->z * lastDirection->z);

            if (lastNorm > 0.01) {
                double dotProduct = dx * lastDirection->x / lastNorm +
                                   dy * lastDirection->y / lastNorm +
                                   dz * lastDirection->z / lastNorm;

                // 方向一致性奖励 [-1, 1] -> [0, 1]
                double consistency = (dotProduct + 1.0) / 2.0;
                score += config_.weightConsistency * consistency;
            }
        }
    }

    return score;
}

void FrontierDetector::addVisitedGoal(const Point3D& goal) {
    visitedGoals_.push_back(goal);
}

void FrontierDetector::clearVisitedGoals() {
    visitedGoals_.clear();
}

void FrontierDetector::addUnreachableGoal(const Point3D& goal) {
    unreachableGoals_.push_back(goal);
    std::cout << "[FrontierDetector] 添加不可达目标: ("
              << goal.x << ", " << goal.y << ", " << goal.z << ")" << std::endl;
}

void FrontierDetector::clearUnreachableGoals() {
    unreachableGoals_.clear();
}

bool FrontierDetector::isUnreachable(const Point3D& point, double threshold) const {
    for (const auto& unreachable : unreachableGoals_) {
        if (point.distanceTo(unreachable) < threshold) {
            return true;
        }
    }
    return false;
}

double FrontierDetector::calculateHistoryPenalty(const Point3D& point) const {
    double penalty = 0.0;

    // 对已访问目标的惩罚（增大范围和力度）
    for (const auto& visited : visitedGoals_) {
        double dist = point.distanceTo(visited);
        if (dist < 3.0) {  // 增大惩罚范围到3米
            penalty += 0.8 * (1.0 - dist / 3.0);  // 增大惩罚力度
        }
    }

    return penalty;
}

double FrontierDetector::calculateTrajectoryPenalty(const Point3D& point) const {
    double penalty = 0.0;

    // 对轨迹历史的惩罚（避免重复走过的路径）
    for (const auto& trajPoint : trajectoryHistory_) {
        double dist = point.distanceTo(trajPoint);
        if (dist < 1.5) {  // 1.5米范围内的轨迹点
            penalty += 0.3 * (1.0 - dist / 1.5);
        }
    }

    // 使用网格查询进行快速惩罚
    int64_t gridKey = pointToGridKey(point, trajectoryGridSize_);
    if (visitedGridCells_.find(gridKey) != visitedGridCells_.end()) {
        penalty += 0.5;  // 已访问过的网格额外惩罚
    }

    return std::min(penalty, 2.0);  // 限制最大惩罚
}

void FrontierDetector::recordTrajectoryPoint(const Point3D& point) {
    // 检查是否与最近的轨迹点距离足够远
    if (!trajectoryHistory_.empty()) {
        double dist = point.distanceTo(trajectoryHistory_.back());
        if (dist < trajectoryGridSize_ * 0.5) {
            return;  // 太近了，不记录
        }
    }

    trajectoryHistory_.push_back(point);

    // 记录到网格
    int64_t gridKey = pointToGridKey(point, trajectoryGridSize_);
    visitedGridCells_.insert(gridKey);

    // 限制轨迹历史长度（保留最近的1000个点）
    if (trajectoryHistory_.size() > 1000) {
        trajectoryHistory_.erase(trajectoryHistory_.begin());
    }
}

void FrontierDetector::clearTrajectoryHistory() {
    trajectoryHistory_.clear();
    visitedGridCells_.clear();
}

void FrontierDetector::setEnvironmentBounds(const Point3D& minBound, const Point3D& maxBound) {
    envMinBound_ = minBound;
    envMaxBound_ = maxBound;
    boundsSet_ = true;
    std::cout << "[FrontierDetector] 环境边界设置: ("
              << minBound.x << ", " << minBound.y << ", " << minBound.z << ") - ("
              << maxBound.x << ", " << maxBound.y << ", " << maxBound.z << ")" << std::endl;
}

void FrontierDetector::updateCoverage(const OctoMapManager& octomapManager) {
    if (!boundsSet_) return;

    // 获取已探索体积
    double exploredVolume = octomapManager.getExploredVolume();

    // 计算环境总体积（考虑高度限制）
    double envWidth = envMaxBound_.x - envMinBound_.x;
    double envDepth = envMaxBound_.y - envMinBound_.y;
    double envHeight = std::min(envMaxBound_.z, config_.maxHeight) -
                       std::max(envMinBound_.z, config_.minHeight);
    envHeight = std::max(envHeight, 0.1);  // 避免除零

    double totalVolume = envWidth * envDepth * envHeight;

    if (totalVolume > 0) {
        explorationCoverage_ = std::min(exploredVolume / totalVolume, 1.0);
    }
}

int64_t FrontierDetector::pointToGridKey(const Point3D& point, double gridSize) const {
    int64_t gx = static_cast<int64_t>(std::floor(point.x / gridSize));
    int64_t gy = static_cast<int64_t>(std::floor(point.y / gridSize));
    int64_t gz = static_cast<int64_t>(std::floor(point.z / gridSize));

    // 组合成唯一键值（假设坐标范围在 ±1000 米内）
    return (gx + 10000) * 100000000LL + (gy + 10000) * 10000LL + (gz + 10000);
}

}  // namespace exploration
