#include "mock_drone.hpp"
#include "octomap_manager.hpp"
#include "frontier_detector.hpp"
#include "path_planner.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>

using namespace exploration;

void printSeparator(const std::string& title = "") {
    std::cout << "\n========================================" << std::endl;
    if (!title.empty()) {
        std::cout << "  " << title << std::endl;
        std::cout << "========================================" << std::endl;
    }
}

int main(int argc, char* argv[]) {
    printSeparator("3D 探索算法离线仿真测试");

    // 解析参数
    std::string pcdFile = "zzxl3.pcd";
    int maxIterations = 50;
    double simDt = 0.1;  // 仿真步长 (秒)
    double resolution = 0.1;  // OctoMap 分辨率

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-p" || arg == "--pcd") && i + 1 < argc) {
            pcdFile = argv[++i];
        } else if ((arg == "-n" || arg == "--iterations") && i + 1 < argc) {
            maxIterations = std::stoi(argv[++i]);
        } else if ((arg == "-r" || arg == "--resolution") && i + 1 < argc) {
            resolution = std::stod(argv[++i]);
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "用法: " << argv[0] << " [选项]\n"
                      << "  -p, --pcd <file>       PCD 文件路径 (默认: zzxl3.pcd)\n"
                      << "  -n, --iterations <N>   最大迭代次数 (默认: 50)\n"
                      << "  -r, --resolution <m>   OctoMap 分辨率 (默认: 0.1)\n"
                      << "  -h, --help             显示帮助\n";
            return 0;
        }
    }

    // ========== 1. 初始化 Mock 无人机 ==========
    printSeparator("1. 加载环境");

    MockDrone drone;

    // 配置传感器
    SensorConfig sensorCfg;
    sensorCfg.horizontalFov = 70.0;   // 水平 70°
    sensorCfg.verticalFov = 60.0;     // 垂直 60°
    sensorCfg.maxRange = 4.0;         // 4米感知距离
    sensorCfg.minRange = 0.2;
    sensorCfg.maxPointsPerFrame = 5000;
    drone.setSensorConfig(sensorCfg);

    // 配置运动学
    DroneKinematicsConfig kinCfg;
    kinCfg.maxVelocity = 0.5;         // 0.5 m/s
    kinCfg.positionTolerance = 0.15;
    drone.setKinematicsConfig(kinCfg);

    // 加载环境
    if (!drone.loadEnvironment(pcdFile)) {
        std::cerr << "无法加载 PCD 文件!" << std::endl;
        return 1;
    }

    // 获取环境边界，设置合理的初始位置
    Point3D minBound, maxBound;
    drone.getEnvironmentBounds(minBound, maxBound);

    std::cout << "环境边界: X[" << minBound.x << ", " << maxBound.x << "] "
              << "Y[" << minBound.y << ", " << maxBound.y << "] "
              << "Z[" << minBound.z << ", " << maxBound.z << "]" << std::endl;

    // 初始位置设在环境中心，高度在 Z 范围中间偏下
    Point3D startPos = {
        (minBound.x + maxBound.x) / 2.0,
        (minBound.y + maxBound.y) / 2.0,
        minBound.z + 1.0  // 离地面 1 米
    };

    // 确保初始位置在合理范围内
    if (startPos.z > maxBound.z - 0.5) {
        startPos.z = (minBound.z + maxBound.z) / 2.0;
    }

    drone.setInitialPose(startPos, 0.0);

    // ========== 2. 初始化 OctoMap ==========
    printSeparator("2. 初始化 OctoMap");

    OctoMapManager octomap(resolution);
    std::cout << "[OctoMap] 分辨率: " << resolution << "m" << std::endl;

    // ========== 3. 初始化前沿点检测器和路径规划器 ==========
    ExplorationConfig config;
    config.resolution = resolution;
    config.minClusterSize = 5;
    config.clusterRadius = 0.5;
    config.astarMaxIterations = 50000;
    config.safetyMargin = 0.2;

    FrontierDetector frontierDetector(config);
    PathPlanner pathPlanner(config);

    // ========== 4. 开始仿真循环 ==========
    printSeparator("3. 开始探索仿真");

    int iteration = 0;
    int waypointsVisited = 0;
    double totalDistance = 0.0;
    Point3D lastPosition = startPos;

    std::cout << std::fixed << std::setprecision(2);

    while (iteration < maxIterations) {
        std::cout << "\n--- 迭代 " << iteration + 1 << " ---" << std::endl;

        // 获取当前位置
        Point3D currentPos = drone.getPosition();
        double currentYaw = drone.getYaw();

        std::cout << "无人机位置: (" << currentPos.x << ", "
                  << currentPos.y << ", " << currentPos.z << ")"
                  << " yaw=" << currentYaw * 180.0 / M_PI << "°" << std::endl;

        // 感知环境
        PointCloud visibleCloud = drone.perceive();
        std::cout << "感知到 " << visibleCloud.points.size() << " 个点" << std::endl;

        if (visibleCloud.points.empty()) {
            std::cout << "警告: 没有感知到点云，可能需要调整位置" << std::endl;
            iteration++;
            continue;
        }

        // 更新 OctoMap
        octomap.insertPointCloud(visibleCloud, currentPos);

        // 打印地图统计
        auto stats = octomap.getStats();
        std::cout << "地图状态 - 占用: " << stats.occupiedNodes
                  << ", 空闲: " << stats.freeNodes
                  << ", 总节点: " << stats.totalNodes << std::endl;

        // 检测前沿点
        auto frontiers = frontierDetector.detectFrontiers(octomap, currentPos);
        std::cout << "检测到 " << frontiers.size() << " 个前沿点簇" << std::endl;

        if (frontiers.empty()) {
            std::cout << "\n探索完成! 没有更多前沿点" << std::endl;
            break;
        }

        // 选择最佳前沿点 (frontiers 已按分数排序)
        const auto& bestFrontier = frontiers[0];
        std::cout << "选择目标: (" << bestFrontier.centroid.x << ", "
                  << bestFrontier.centroid.y << ", " << bestFrontier.centroid.z
                  << ") 分数=" << bestFrontier.score
                  << " 大小=" << bestFrontier.cells.size() << std::endl;

        // 路径规划
        Path3D path = pathPlanner.planPath(currentPos, bestFrontier.centroid, octomap);

        if (!path.isValid || path.waypoints.empty()) {
            std::cout << "路径规划失败，标记为不可达" << std::endl;
            frontierDetector.addUnreachableGoal(bestFrontier.centroid);
            iteration++;
            continue;
        }

        std::cout << "规划路径: " << path.waypoints.size()
                  << " 个路点, 长度=" << path.totalLength << "m" << std::endl;

        // 模拟沿路径飞行
        for (const auto& waypoint : path.waypoints) {
            drone.setTargetPosition(waypoint);

            // 仿真运动直到到达
            int simSteps = 0;
            while (!drone.hasReachedTarget() && simSteps < 100) {
                drone.update(simDt);
                simSteps++;
            }
        }

        // 更新统计
        Point3D newPos = drone.getPosition();
        double dist = std::sqrt(
            std::pow(newPos.x - lastPosition.x, 2) +
            std::pow(newPos.y - lastPosition.y, 2) +
            std::pow(newPos.z - lastPosition.z, 2)
        );
        totalDistance += dist;
        lastPosition = newPos;
        waypointsVisited++;

        // 更新历史
        frontierDetector.addVisitedGoal(newPos);

        iteration++;
    }

    // ========== 5. 输出结果 ==========
    printSeparator("4. 仿真结果");

    std::cout << "迭代次数: " << iteration << std::endl;
    std::cout << "访问路点: " << waypointsVisited << std::endl;
    std::cout << "总飞行距离: " << totalDistance << " m" << std::endl;

    auto finalStats = octomap.getStats();
    std::cout << "最终地图 - 占用: " << finalStats.occupiedNodes
              << ", 空闲: " << finalStats.freeNodes
              << ", 内存: " << finalStats.memoryUsage << " MB" << std::endl;

    // 保存 OctoMap
    std::string outputFile = "exploration_result.bt";
    if (octomap.saveToFile(outputFile)) {
        std::cout << "\n地图已保存到: " << outputFile << std::endl;
        std::cout << "可以使用 octovis 查看: octovis " << outputFile << std::endl;
    }

    printSeparator();
    std::cout << "仿真完成!" << std::endl;

    return 0;
}
