#pragma once

#include "types.hpp"
#include <octomap/OcTree.h>
#include <memory>
#include <mutex>

namespace exploration {

/**
 * OctoMap 3D地图管理器
 * 负责点云积累、体素地图更新、碰撞检测
 */
class OctoMapManager {
public:
    explicit OctoMapManager(double resolution = 0.1);
    ~OctoMapManager();

    // 禁止拷贝
    OctoMapManager(const OctoMapManager&) = delete;
    OctoMapManager& operator=(const OctoMapManager&) = delete;

    // ============================================================
    // 地图更新
    // ============================================================

    /**
     * 插入点云（光线追踪更新）
     * @param cloud 点云数据
     * @param sensorOrigin 传感器原点（无人机位置）
     */
    void insertPointCloud(const PointCloud& cloud, const Point3D& sensorOrigin);

    /**
     * 批量插入点（不做光线追踪，仅标记占据）
     */
    void insertOccupiedPoints(const std::vector<Point3D>& points);

    /**
     * 重置地图
     */
    void reset();

    // ============================================================
    // 查询接口
    // ============================================================

    /**
     * 查询体素占据状态
     * @return 1=空闲, -1=占据, 0=未知
     */
    int getOccupancy(const Point3D& point) const;
    int getOccupancy(double x, double y, double z) const;

    /**
     * 检查点是否在障碍物内（考虑安全边距）
     * 未知空间也视为障碍物
     */
    bool isInCollision(const Point3D& point, double safetyMargin = 0.0) const;

    /**
     * 检查点是否在已知占据障碍物内（忽略未知空间）
     * 用于检查起点是否安全（无人机已经在那里了）
     */
    bool isOccupied(const Point3D& point, double safetyMargin = 0.0) const;

    /**
     * 检查两点之间的直线路径是否畅通
     */
    bool isPathClear(const Point3D& start, const Point3D& end,
                     double stepSize = 0.1, double safetyMargin = 0.0) const;

    /**
     * 获取点周围的障碍物密度
     * @param radius 检测半径
     * @return 密度值 [0, 1]
     */
    double getObstacleDensity(const Point3D& center, double radius) const;

    // ============================================================
    // 前沿点检测
    // ============================================================

    /**
     * 检测所有前沿体素
     * 前沿体素：空闲体素且有未知邻居
     * @return 前沿点列表
     */
    std::vector<Point3D> detectFrontierCells() const;

    /**
     * 检查体素是否为前沿
     */
    bool isFrontierCell(const octomap::OcTreeKey& key) const;

    // ============================================================
    // 地图信息
    // ============================================================

    /**
     * 获取已探索体积 (立方米)
     */
    double getExploredVolume() const;

    /**
     * 获取地图边界
     */
    void getBounds(Point3D& minBound, Point3D& maxBound) const;

    /**
     * 获取地图统计信息
     */
    struct MapStats {
        size_t totalNodes = 0;
        size_t occupiedNodes = 0;
        size_t freeNodes = 0;
        double exploredVolume = 0.0;
        double memoryUsage = 0.0;  // MB
    };
    MapStats getStats() const;

    /**
     * 获取OctoMap原始指针（用于高级操作）
     */
    octomap::OcTree* getOcTree() { return octree_.get(); }
    const octomap::OcTree* getOcTree() const { return octree_.get(); }

    /**
     * 获取分辨率
     */
    double getResolution() const { return resolution_; }

    // ============================================================
    // 保存/加载
    // ============================================================

    bool saveToFile(const std::string& filename) const;
    bool loadFromFile(const std::string& filename);

private:
    double resolution_;
    std::unique_ptr<octomap::OcTree> octree_;
    mutable std::mutex mutex_;

    // 26邻域偏移
    static constexpr int NUM_NEIGHBORS = 26;
    static const int neighborOffsets_[26][3];
};

}  // namespace exploration
