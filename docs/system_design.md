# 3D 室内无人机自主探索系统设计文档

## 一、系统概述

本系统是一个基于 **前沿探索（Frontier-based Exploration）** 的室内无人机 3D 自主探索仿真系统。使用 OctoMap 构建三维体素地图，通过检测已知空间与未知空间的边界（前沿点），引导无人机逐步探索整个环境。

### 技术栈
- **语言**: C++17
- **机器人框架**: ROS1 Noetic
- **3D地图**: OctoMap (八叉树)
- **点云处理**: PCL (Point Cloud Library)
- **可视化**: RViz
- **矩阵运算**: Eigen3

---

## 二、系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                      sim_node (ROS节点)                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐  ┌────────────────┐  ┌──────────────────┐    │
│  │  MockDrone   │  │ OctoMapManager │  │ FrontierDetector │    │
│  │  (模拟无人机) │  │  (3D体素地图)   │  │   (前沿点检测)    │    │
│  └──────┬───────┘  └───────┬────────┘  └────────┬─────────┘    │
│         │                  │                     │              │
│         │    感知点云       │    地图查询          │  前沿点      │
│         └─────────────────►│◄────────────────────┘              │
│                            │                                    │
│                            ▼                                    │
│                   ┌──────────────┐                              │
│                   │ PathPlanner  │                              │
│                   │  (3D A*规划)  │                              │
│                   └──────────────┘                              │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼ ROS Topics
              ┌───────────────┴───────────────┐
              │                               │
        ┌─────┴─────┐                   ┌─────┴─────┐
        │   RViz    │                   │  其他节点  │
        │  (可视化)  │                   │           │
        └───────────┘                   └───────────┘
```

---

## 三、核心组件详解

### 3.1 MockDrone (模拟无人机)

**文件位置**: `src/mock_drone.cpp`, `include/mock_drone.hpp`

#### 功能
模拟真实无人机的感知和运动能力，从预加载的 PCD 点云文件中提取"可见"点云。

#### 主要参数
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `horizontalFov` | 70° | 水平视场角 |
| `verticalFov` | 60° | 垂直视场角 |
| `maxRange` | 4.0m | 最大感知距离 |
| `minRange` | 0.2m | 最小感知距离 |
| `maxPointsPerFrame` | 5000 | 每帧最大点数 |
| `maxVelocity` | 0.5m/s | 最大飞行速度 |
| `positionTolerance` | 0.15m | 到达判定容差 |

#### 核心算法 - 感知 (perceive)
```cpp
PointCloud perceive() {
    // 1. 使用 KD-Tree 搜索 maxRange 内的所有环境点
    kdtree_.radiusSearch(dronePosition, maxRange, indices, distances);

    // 2. 视锥体裁剪
    for (每个候选点) {
        // 计算点相对于无人机的方向
        向量 = 点位置 - 无人机位置

        // 转换到无人机坐标系（考虑yaw角）
        局部坐标 = 旋转矩阵 * 向量

        // 检查水平角度是否在 FOV 内
        水平角 = atan2(局部y, 局部x)
        if (|水平角| > horizontalFov/2) continue;

        // 检查垂直角度是否在 FOV 内
        垂直角 = atan2(局部z, sqrt(局部x² + 局部y²))
        if (|垂直角| > verticalFov/2) continue;

        // 通过检查，加入可见点云
        visibleCloud.add(点);
    }

    return visibleCloud;
}
```

---

### 3.2 OctoMapManager (八叉树地图管理器)

**文件位置**: `src/octomap_manager.cpp`, `include/octomap_manager.hpp`

#### 功能
管理 3D 体素地图，支持点云插入、碰撞检测、前沿点检测。

#### 体素状态
| 状态 | 值 | 说明 |
|------|-----|------|
| 未知 (Unknown) | 0 | 从未被观测过的区域 |
| 空闲 (Free) | 1 | 射线穿过的区域 |
| 占据 (Occupied) | -1 | 点云击中的位置 |

#### 核心函数

##### 1. 点云插入 (光线追踪)
```cpp
void insertPointCloud(cloud, sensorOrigin) {
    // OctoMap 内置的光线追踪算法
    // 从 sensorOrigin 到每个点画射线：
    //   - 射线经过的体素 → 标记为空闲
    //   - 射线终点的体素 → 标记为占据
    octree_->insertPointCloud(cloud, origin);
}
```

##### 2. 碰撞检测 (isInCollision)
```cpp
// 未知空间也视为障碍物
bool isInCollision(point, safetyMargin) {
    if (safetyMargin <= 0) {
        node = octree_->search(point);
        // 未知(nullptr) 或 占据 都返回 true
        return node == nullptr || isOccupied(node);
    }

    // 有安全边距时，检查球形区域内所有体素
    for (球形区域内的每个体素) {
        if (未知 || 占据) return true;
    }
    return false;
}
```

##### 3. 占据检测 (isOccupied)
```cpp
// 忽略未知空间，只检查已知障碍物
bool isOccupied(point, safetyMargin) {
    node = octree_->search(point);
    // 只有已知且占据才返回 true
    return node != nullptr && isOccupied(node);
}
```

> **设计原因**:
> - `isInCollision`: 用于路径规划，保守策略，不能飞入未知区域
> - `isOccupied`: 用于检查起点，无人机已在那里，未知不影响

##### 4. 前沿点检测
```cpp
vector<Point3D> detectFrontierCells() {
    for (每个叶子节点) {
        if (节点是占据的) continue;  // 跳过障碍物

        // 检查26邻域
        for (26个邻居方向) {
            邻居节点 = search(邻居位置);
            if (邻居节点 == nullptr) {  // 未知
                // 当前节点是前沿点！
                frontiers.add(当前节点位置);
                break;
            }
        }
    }
    return frontiers;
}
```

---

### 3.3 FrontierDetector (前沿点检测器)

**文件位置**: `src/frontier_detector.cpp`, `include/frontier_detector.hpp`

#### 前沿点定义
> **前沿点 (Frontier)**: 已知空闲空间与未知空间的边界体素

#### 处理流程
```
原始前沿体素 → 距离/高度过滤 → 聚类 → 评分排序 → 输出前沿簇
```

#### 聚类算法
```cpp
vector<FrontierCluster> clusterFrontiers(cells, clusterRadius, minSize) {
    for (每个未访问的前沿体素 i) {
        创建新簇 cluster;
        cluster.add(cells[i]);

        // 贪婪聚类：找所有距离 < clusterRadius 的点
        for (每个其他未访问体素 j) {
            if (cells[j] 与簇内任意点距离 < clusterRadius) {
                cluster.add(cells[j]);
                标记 j 为已访问;
            }
        }

        // 过滤太小的簇
        if (cluster.size >= minSize) {
            计算簇中心 (centroid);
            clusters.add(cluster);
        }
    }
    return clusters;
}
```

#### 评分函数
```cpp
double calculateScore(cluster, currentPos, lastDirection) {
    score = 0;

    // 1. 信息增益（簇越大，信息增益越高）
    score += weightInfoGain * min(cluster.size / 50.0, 1.0);

    // 2. 距离评分（最佳距离约3米）
    distance = cluster.centroid.distanceTo(currentPos);
    distScore = 1.0 - |distance - 3.0| / 3.0;
    score += weightDistance * max(0, distScore);

    // 3. 方向一致性（与上次飞行方向一致更好）
    if (lastDirection != null) {
        dotProduct = normalize(方向向量) · normalize(lastDirection);
        consistency = (dotProduct + 1.0) / 2.0;  // 映射到 [0, 1]
        score += weightConsistency * consistency;
    }

    // 4. 太近的前沿点给低分（< 0.5m）
    if (distance < 0.5) return -1000;

    return score;
}
```

---

### 3.4 PathPlanner (路径规划器)

**文件位置**: `src/path_planner.cpp`, `include/path_planner.hpp`

#### 算法
3D A* 搜索，26邻域扩展

#### 邻域定义
```cpp
// 26邻域 = 6面 + 12边 + 8角
const int neighbors[26][3] = {
    // 6个面邻居 (cost = 1.0)
    {-1,0,0}, {1,0,0}, {0,-1,0}, {0,1,0}, {0,0,-1}, {0,0,1},

    // 12个边邻居 (cost = √2 ≈ 1.414)
    {-1,-1,0}, {-1,1,0}, {1,-1,0}, {1,1,0},
    {-1,0,-1}, {-1,0,1}, {1,0,-1}, {1,0,1},
    {0,-1,-1}, {0,-1,1}, {0,1,-1}, {0,1,1},

    // 8个角邻居 (cost = √3 ≈ 1.732)
    {-1,-1,-1}, {-1,-1,1}, {-1,1,-1}, {-1,1,1},
    {1,-1,-1}, {1,-1,1}, {1,1,-1}, {1,1,1}
};
```

#### A* 核心流程
```cpp
Path3D planPath(start, goal, octomap) {
    // 0. 先尝试直线路径
    if (isDirectPathClear(start, goal)) {
        return 直线路径;
    }

    // 1. 检查起点（用 isOccupied，忽略未知）
    if (octomap.isOccupied(start, safetyMargin)) {
        return 失败("起点在障碍物内");
    }

    // 2. 检查终点（用 isInCollision，未知=障碍）
    if (octomap.isInCollision(goal, safetyMargin)) {
        return 失败("终点不可通行");
    }

    // 3. A* 搜索
    openSet.push({start, g=0, f=heuristic(start, goal)});

    while (!openSet.empty() && iterations < maxIterations) {
        current = openSet.pop();  // 取 f 最小的节点

        if (current == goal) {
            return 重建路径(cameFrom, goal);
        }

        // 扩展26个邻居
        for (i = 0; i < 26; i++) {
            neighbor = current + neighbors[i];

            // 检查可通行性
            if (!isTraversable(neighbor)) continue;

            // 对角线移动：检查相邻格子（防穿墙）
            if (是对角线移动) {
                if (!相邻格子可通行) continue;
            }

            tentativeG = current.g + neighborCosts[i] * resolution;

            if (tentativeG < gScore[neighbor]) {
                gScore[neighbor] = tentativeG;
                cameFrom[neighbor] = current;
                f = tentativeG + heuristic(neighbor, goal);
                openSet.push({neighbor, g=tentativeG, f=f});
            }
        }
    }

    return 失败("搜索超时");
}
```

#### 观察点搜索 (findObservationPoint)
```cpp
// 问题：前沿点在未知空间边界，直接规划会失败
// 解决：找一个在已知空闲空间的观察点

Point3D findObservationPoint(frontier, currentPos, octomap) {
    direction = normalize(frontier - currentPos);
    totalDist = distance(frontier, currentPos);

    lastValidPoint = null;

    // 从当前位置向前沿点方向走
    for (d = step; d <= totalDist; d += step) {
        candidate = currentPos + direction * d;

        if (!octomap.isInCollision(candidate)) {
            // 可通行，记录
            lastValidPoint = candidate;
        } else {
            // 遇到障碍或未知，停止
            break;
        }
    }

    if (lastValidPoint != null) {
        return lastValidPoint;  // 返回最后一个可通行点
    }

    // 如果一步都走不了，原地观察
    if (!octomap.isOccupied(currentPos)) {
        return currentPos;
    }

    return 无效;
}
```

---

## 四、主循环流程

**文件位置**: `ros/exploration_sim/src/sim_node.cpp`

### 初始化阶段
```
1. 加载 PCD 环境文件
2. 配置传感器参数（FOV, 感知距离等）
3. 设置初始位置（环境中心，高度1.5m）
4. 创建 OctoMap (分辨率 0.1m)
5. 创建 FrontierDetector 和 PathPlanner
6. 等待 RViz 连接
```

### 探索循环
```
┌─────────────────────────────────────────────────────────────┐
│                    每次迭代 (2Hz)                            │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ 1. 感知 (Perceive)                                   │   │
│  │    drone.perceive() → 获取当前视野内的点云            │   │
│  │    如果没看到点 → 原地旋转45°，下次再试              │   │
│  └─────────────────────────────────────────────────────┘   │
│                           ▼                                 │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ 2. 建图 (Mapping)                                    │   │
│  │    accumulateCloud() → 累积点云（用于可视化）         │   │
│  │    octomap.insertPointCloud() → 光线追踪更新地图     │   │
│  └─────────────────────────────────────────────────────┘   │
│                           ▼                                 │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ 3. 前沿检测 (Frontier Detection)                     │   │
│  │    frontierDetector.detectFrontiers()                │   │
│  │    → 找所有 (空闲 + 有未知邻居) 的体素               │   │
│  │    → 聚类、评分、排序                                │   │
│  │    如果无前沿点 → 探索完成，退出                     │   │
│  └─────────────────────────────────────────────────────┘   │
│                           ▼                                 │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ 4. 目标选择 (Goal Selection)                         │   │
│  │    for (前5个最优前沿点) {                           │   │
│  │        观察点 = findObservationPoint(前沿点)         │   │
│  │        路径 = planPath(当前位置, 观察点)             │   │
│  │        if (路径有效) break;                          │   │
│  │        else 标记为不可达;                            │   │
│  │    }                                                 │   │
│  │    如果全部不可达 → 跳过本次迭代                     │   │
│  └─────────────────────────────────────────────────────┘   │
│                           ▼                                 │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ 5. 执行飞行 (Navigation)                             │   │
│  │    for (路径上每个路点) {                            │   │
│  │        drone.setTargetPosition(路点);                │   │
│  │        while (!到达) {                               │   │
│  │            drone.update(dt);                         │   │
│  │            发布位姿、轨迹、TF;                       │   │
│  │            每10步感知一次，更新地图;  // 边飞边看    │   │
│  │        }                                             │   │
│  │    }                                                 │   │
│  └─────────────────────────────────────────────────────┘   │
│                           ▼                                 │
│                      下一次迭代                             │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

---

## 五、ROS 接口

### 5.1 发布的话题

| 话题名 | 消息类型 | 频率 | 说明 |
|--------|----------|------|------|
| `/drone/pose` | `geometry_msgs/PoseStamped` | 实时 | 无人机当前位姿 |
| `/drone/marker` | `visualization_msgs/Marker` | 实时 | 无人机可视化（箭头） |
| `/drone/trajectory` | `nav_msgs/Path` | 每次迭代 | 历史飞行轨迹 |
| `/drone/current_path` | `nav_msgs/Path` | 每次迭代 | 当前规划路径 |
| `/octomap` | `octomap_msgs/Octomap` | 每次迭代 | 八叉树地图 |
| `/frontiers` | `visualization_msgs/MarkerArray` | 每次迭代 | 前沿点（红=最优，蓝=其他） |
| `/accumulated_cloud` | `sensor_msgs/PointCloud2` | Latched | 累积探索点云 |
| `/visible_cloud` | `sensor_msgs/PointCloud2` | 每次迭代 | 当前可见点云 |

### 5.2 TF 变换
```
world
  └── drone
```

### 5.3 Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pcd_file` | `zzxl3.pcd` | 环境点云文件路径 |
| `max_iterations` | 100 | 最大探索迭代次数 |
| `resolution` | 0.1 | OctoMap 分辨率 (米) |
| `sim_rate` | 2.0 | 仿真频率 (Hz) |

---

## 六、关键设计决策

### 6.1 未知空间处理策略

| 场景 | 使用函数 | 对未知的处理 | 原因 |
|------|----------|--------------|------|
| 检查起点 | `isOccupied()` | 忽略 | 无人机已在此位置，肯定安全 |
| 检查终点 | `isInCollision()` | 视为障碍 | 不能飞入未知区域 |
| 路径搜索 | `isInCollision()` | 视为障碍 | 保守策略，只在已知空闲区域规划 |
| 找观察点 | `findObservationPoint()` | 在边界前停止 | 飞到已知区域边缘观察 |

### 6.2 为什么需要 findObservationPoint？

**问题**: 前沿点定义为"空闲且有未知邻居"，但当 `isInCollision` 将未知视为障碍后，前沿点本身可能被判定为不可通行（因为它旁边就是未知）。

**解决**: 不直接规划到前沿点，而是：
1. 从当前位置向前沿点方向走
2. 记录路上最后一个可通行点
3. 规划到该观察点
4. 到达后感知，扩展已知区域

### 6.3 边飞边感知

在执行路径时，每10个仿真步骤感知一次并更新地图：
- 及时发现新障碍物
- 扩展已知区域
- 为下一次规划提供更多信息

---

## 七、配置参数汇总

### 7.1 传感器配置 (SensorConfig)
```cpp
horizontalFov = 70.0;      // 水平视场角 (度)
verticalFov = 60.0;        // 垂直视场角 (度)
maxRange = 4.0;            // 最大感知距离 (米)
minRange = 0.2;            // 最小感知距离 (米)
maxPointsPerFrame = 5000;  // 每帧最大点数
```

### 7.2 运动学配置 (DroneKinematicsConfig)
```cpp
maxVelocity = 0.5;         // 最大速度 (m/s)
positionTolerance = 0.15;  // 到达判定容差 (米)
```

### 7.3 探索配置 (ExplorationConfig)
```cpp
resolution = 0.1;              // 地图分辨率 (米)
safetyMargin = 0.2;            // 安全边距 (米)
minClusterSize = 5;            // 最小簇大小
clusterRadius = 0.5;           // 聚类半径 (米)
maxExplorationDistance = 10.0; // 最大探索距离 (米)
minHeight = 0.5;               // 最小飞行高度 (米)
maxHeight = 3.0;               // 最大飞行高度 (米)
astarMaxIterations = 50000;    // A*最大迭代次数
```

---

## 八、运行指南

### 8.1 编译
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 8.2 启动仿真
```bash
# 终端1: 启动仿真节点
roslaunch exploration_sim sim_only.launch

# 终端2: 启动 RViz (虚拟机需要软件渲染)
export LIBGL_ALWAYS_SOFTWARE=1
rosrun rviz rviz -d ~/catkin_ws/src/exploration-3d/ros/exploration_sim/rviz/exploration.rviz
```

### 8.3 自定义参数
```bash
roslaunch exploration_sim sim_only.launch \
    pcd_file:=/path/to/your/environment.pcd \
    max_iterations:=200 \
    resolution:=0.05 \
    sim_rate:=5.0
```

---

## 九、文件结构

```
exploration-3d/
├── include/
│   ├── types.hpp              # 基础类型定义 (Point3D, PointCloud, Path3D)
│   ├── mock_drone.hpp         # 模拟无人机
│   ├── octomap_manager.hpp    # OctoMap 管理器
│   ├── frontier_detector.hpp  # 前沿点检测器
│   └── path_planner.hpp       # 路径规划器
├── src/
│   ├── mock_drone.cpp
│   ├── octomap_manager.cpp
│   ├── frontier_detector.cpp
│   └── path_planner.cpp
├── ros/
│   └── exploration_sim/
│       ├── src/
│       │   └── sim_node.cpp   # ROS 仿真节点
│       ├── launch/
│       │   ├── sim.launch     # 带 RViz 启动
│       │   └── sim_only.launch # 仅仿真节点
│       ├── rviz/
│       │   └── exploration.rviz # RViz 配置
│       ├── CMakeLists.txt
│       └── package.xml
├── docs/
│   └── system_design.md       # 本文档
├── zzxl3.pcd                  # 示例环境点云
├── CMakeLists.txt
└── README.md
```

---

## 十、后续改进方向

1. **多无人机协同探索**: 分配不同前沿点给不同无人机
2. **动态障碍物处理**: 实时检测并避开移动障碍物
3. **更优的前沿评分**: 考虑可见性、信息熵等因素
4. **路径平滑**: 使用 B-spline 或多项式轨迹优化
5. **真实传感器接入**: 替换 MockDrone 为真实深度相机/激光雷达
6. **ROS2 迁移**: 适配 ROS2 Humble/Iron
