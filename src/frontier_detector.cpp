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

    // 4. 计算评分并排序
    for (auto& cluster : clusters) {
        cluster.score = calculateScore(cluster, currentPos, nullptr);
    }

    std::sort(clusters.begin(), clusters.end(),
              [](const FrontierCluster& a, const FrontierCluster& b) {
                  return a.score > b.score;
              });

    std::cout << "[FrontierDetector] 检测到 " << clusters.size()
              << " 个前沿簇 (原始体素: " << frontierCells.size()
              << ", 过滤后: " << filteredCells.size() << ")" << std::endl;

    return clusters;
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

    // 1. 信息增益
    score += config_.weightInfoGain * cluster.infoGain;

    // 2. 距离成本（越近越好）
    double distance = cluster.centroid.distanceTo(currentPos);
    double distanceCost = 1.0 / (1.0 + distance);
    score += config_.weightDistance * distanceCost;

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

    for (const auto& visited : visitedGoals_) {
        double dist = point.distanceTo(visited);
        if (dist < 2.0) {
            penalty += 0.5 * (1.0 - dist / 2.0);
        }
    }

    return penalty;
}

}  // namespace exploration
