#include "octomap_manager.hpp"
#include <iostream>
#include <cmath>

namespace exploration {

// 26邻域偏移（3D）
const int OctoMapManager::neighborOffsets_[26][3] = {
    // 6个面邻居
    {-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1},
    // 12个边邻居
    {-1, -1, 0}, {-1, 1, 0}, {1, -1, 0}, {1, 1, 0},
    {-1, 0, -1}, {-1, 0, 1}, {1, 0, -1}, {1, 0, 1},
    {0, -1, -1}, {0, -1, 1}, {0, 1, -1}, {0, 1, 1},
    // 8个角邻居
    {-1, -1, -1}, {-1, -1, 1}, {-1, 1, -1}, {-1, 1, 1},
    {1, -1, -1}, {1, -1, 1}, {1, 1, -1}, {1, 1, 1}
};

OctoMapManager::OctoMapManager(double resolution)
    : resolution_(resolution)
    , octree_(std::make_unique<octomap::OcTree>(resolution))
{
    // 设置占据阈值
    octree_->setOccupancyThres(0.5);
    octree_->setProbHit(0.7);
    octree_->setProbMiss(0.4);
    octree_->setClampingThresMin(0.12);
    octree_->setClampingThresMax(0.97);

    std::cout << "[OctoMapManager] 初始化完成, 分辨率=" << resolution_ << "m" << std::endl;
}

OctoMapManager::~OctoMapManager() = default;

// ============================================================
// 地图更新
// ============================================================

void OctoMapManager::insertPointCloud(const PointCloud& cloud, const Point3D& sensorOrigin) {
    if (cloud.points.empty()) return;

    std::lock_guard<std::mutex> lock(mutex_);

    // 转换为OctoMap点云格式
    octomap::Pointcloud octoCloud;
    octoCloud.reserve(cloud.points.size());

    for (const auto& p : cloud.points) {
        // 过滤无效点
        if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) continue;
        if (std::isinf(p.x) || std::isinf(p.y) || std::isinf(p.z)) continue;

        octoCloud.push_back(octomap::point3d(p.x, p.y, p.z));
    }

    // 传感器原点
    octomap::point3d origin(sensorOrigin.x, sensorOrigin.y, sensorOrigin.z);

    // 光线追踪插入（自动更新空闲和占据）
    octree_->insertPointCloud(octoCloud, origin);
}

void OctoMapManager::insertOccupiedPoints(const std::vector<Point3D>& points) {
    std::lock_guard<std::mutex> lock(mutex_);

    for (const auto& p : points) {
        octree_->updateNode(octomap::point3d(p.x, p.y, p.z), true);
    }
}

void OctoMapManager::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    octree_->clear();
    std::cout << "[OctoMapManager] 地图已重置" << std::endl;
}

// ============================================================
// 查询接口
// ============================================================

int OctoMapManager::getOccupancy(const Point3D& point) const {
    return getOccupancy(point.x, point.y, point.z);
}

int OctoMapManager::getOccupancy(double x, double y, double z) const {
    std::lock_guard<std::mutex> lock(mutex_);

    octomap::OcTreeNode* node = octree_->search(x, y, z);

    if (node == nullptr) {
        return 0;  // 未知
    }

    if (octree_->isNodeOccupied(node)) {
        return -1;  // 占据
    } else {
        return 1;   // 空闲
    }
}

bool OctoMapManager::isInCollision(const Point3D& point, double safetyMargin) const {
    std::lock_guard<std::mutex> lock(mutex_);

    if (safetyMargin <= 0) {
        // 无安全边距，只检查单点
        octomap::OcTreeNode* node = octree_->search(point.x, point.y, point.z);
        return node != nullptr && octree_->isNodeOccupied(node);
    }

    // 有安全边距，检查球形区域
    double step = resolution_;
    for (double dx = -safetyMargin; dx <= safetyMargin; dx += step) {
        for (double dy = -safetyMargin; dy <= safetyMargin; dy += step) {
            for (double dz = -safetyMargin; dz <= safetyMargin; dz += step) {
                double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                if (dist > safetyMargin) continue;

                octomap::OcTreeNode* node = octree_->search(
                    point.x + dx, point.y + dy, point.z + dz);
                if (node != nullptr && octree_->isNodeOccupied(node)) {
                    return true;
                }
            }
        }
    }

    return false;
}

bool OctoMapManager::isPathClear(const Point3D& start, const Point3D& end,
                                  double stepSize, double safetyMargin) const {
    double dist = start.distanceTo(end);
    if (dist < stepSize) {
        return !isInCollision(end, safetyMargin);
    }

    int steps = static_cast<int>(dist / stepSize) + 1;
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        Point3D p;
        p.x = start.x + t * (end.x - start.x);
        p.y = start.y + t * (end.y - start.y);
        p.z = start.z + t * (end.z - start.z);

        if (isInCollision(p, safetyMargin)) {
            return false;
        }
    }

    return true;
}

double OctoMapManager::getObstacleDensity(const Point3D& center, double radius) const {
    std::lock_guard<std::mutex> lock(mutex_);

    int totalCells = 0;
    int occupiedCells = 0;

    double step = resolution_;
    for (double dx = -radius; dx <= radius; dx += step) {
        for (double dy = -radius; dy <= radius; dy += step) {
            for (double dz = -radius; dz <= radius; dz += step) {
                double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                if (dist > radius) continue;

                totalCells++;

                octomap::OcTreeNode* node = octree_->search(
                    center.x + dx, center.y + dy, center.z + dz);
                if (node != nullptr && octree_->isNodeOccupied(node)) {
                    occupiedCells++;
                }
            }
        }
    }

    if (totalCells == 0) return 0.0;
    return static_cast<double>(occupiedCells) / totalCells;
}

// ============================================================
// 前沿点检测
// ============================================================

std::vector<Point3D> OctoMapManager::detectFrontierCells() const {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<Point3D> frontiers;

    // 遍历所有叶子节点
    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
        // 只检查空闲节点
        if (octree_->isNodeOccupied(*it)) continue;

        octomap::OcTreeKey key = it.getKey();

        if (isFrontierCell(key)) {
            octomap::point3d coord = octree_->keyToCoord(key);
            frontiers.emplace_back(coord.x(), coord.y(), coord.z());
        }
    }

    return frontiers;
}

bool OctoMapManager::isFrontierCell(const octomap::OcTreeKey& key) const {
    // 检查26邻域是否有未知体素
    for (int i = 0; i < NUM_NEIGHBORS; ++i) {
        octomap::OcTreeKey neighborKey(
            key[0] + neighborOffsets_[i][0],
            key[1] + neighborOffsets_[i][1],
            key[2] + neighborOffsets_[i][2]
        );

        octomap::OcTreeNode* node = octree_->search(neighborKey);
        if (node == nullptr) {
            // 有未知邻居 → 是前沿
            return true;
        }
    }

    return false;
}

// ============================================================
// 地图信息
// ============================================================

double OctoMapManager::getExploredVolume() const {
    std::lock_guard<std::mutex> lock(mutex_);

    size_t knownNodes = 0;
    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
        knownNodes++;
    }

    double cellVolume = std::pow(resolution_, 3);
    return knownNodes * cellVolume;
}

void OctoMapManager::getBounds(Point3D& minBound, Point3D& maxBound) const {
    std::lock_guard<std::mutex> lock(mutex_);

    double minX, minY, minZ, maxX, maxY, maxZ;
    octree_->getMetricMin(minX, minY, minZ);
    octree_->getMetricMax(maxX, maxY, maxZ);

    minBound = Point3D(minX, minY, minZ);
    maxBound = Point3D(maxX, maxY, maxZ);
}

OctoMapManager::MapStats OctoMapManager::getStats() const {
    std::lock_guard<std::mutex> lock(mutex_);

    MapStats stats;
    stats.totalNodes = octree_->calcNumNodes();

    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
        if (octree_->isNodeOccupied(*it)) {
            stats.occupiedNodes++;
        } else {
            stats.freeNodes++;
        }
    }

    stats.exploredVolume = (stats.occupiedNodes + stats.freeNodes) * std::pow(resolution_, 3);
    stats.memoryUsage = octree_->memoryUsage() / (1024.0 * 1024.0);

    return stats;
}

// ============================================================
// 保存/加载
// ============================================================

bool OctoMapManager::saveToFile(const std::string& filename) const {
    std::lock_guard<std::mutex> lock(mutex_);

    bool success = octree_->writeBinary(filename);
    if (success) {
        std::cout << "[OctoMapManager] 地图已保存到: " << filename << std::endl;
    } else {
        std::cerr << "[OctoMapManager] 保存地图失败: " << filename << std::endl;
    }
    return success;
}

bool OctoMapManager::loadFromFile(const std::string& filename) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto loadedTree = std::make_unique<octomap::OcTree>(filename);
    if (loadedTree->size() == 0) {
        std::cerr << "[OctoMapManager] 加载地图失败: " << filename << std::endl;
        return false;
    }

    octree_ = std::move(loadedTree);
    resolution_ = octree_->getResolution();

    std::cout << "[OctoMapManager] 地图已加载: " << filename
              << ", 节点数=" << octree_->calcNumNodes()
              << ", 分辨率=" << resolution_ << "m" << std::endl;

    return true;
}

}  // namespace exploration
