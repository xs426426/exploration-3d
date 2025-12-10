# Exploration-3D

纯 C++ 实现的无人机 3D 自主探索系统，基于 OctoMap 体素地图和前沿点探索算法。

## 功能特性

- **3D 体素地图** - 基于 OctoMap 的八叉树地图管理
- **3D 前沿点检测** - 26邻域前沿点检测与聚类
- **3D A* 路径规划** - 支持26方向搜索，带路径简化
- **MQTT 通信** - Protobuf 序列化，与无人机系统通信
- **YAML 配置** - 灵活的参数配置

## 项目结构

```
exploration-3d/
├── CMakeLists.txt              # CMake 构建配置
├── config/
│   └── exploration_config.yaml # 默认配置文件
├── include/
│   ├── types.hpp               # 核心类型定义
│   ├── octomap_manager.hpp     # OctoMap 3D地图管理器
│   ├── frontier_detector.hpp   # 3D前沿点检测器
│   ├── path_planner.hpp        # 3D A*路径规划器
│   ├── mqtt_client.hpp         # MQTT通信客户端
│   ├── exploration_engine.hpp  # 探索引擎主类
│   └── config_loader.hpp       # YAML配置加载器
├── src/
│   ├── main.cpp                # 程序入口
│   ├── octomap_manager.cpp
│   ├── frontier_detector.cpp
│   ├── path_planner.cpp
│   ├── mqtt_client.cpp
│   ├── exploration_engine.cpp
│   └── config_loader.cpp
├── proto/
│   └── drone_message.proto     # Protobuf消息定义
└── scripts/
    ├── build.sh                # 构建脚本
    └── install_deps.sh         # 依赖安装脚本
```

## 依赖

- Ubuntu 20.04 / ROS Noetic
- CMake >= 3.16
- OctoMap
- Eigen3
- Protobuf
- Paho MQTT C/C++
- yaml-cpp

## 编译

```bash
# 安装依赖 (Ubuntu 20.04)
chmod +x scripts/*.sh
./scripts/install_deps.sh

# 编译
./scripts/build.sh

# 或手动编译
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

## 运行

```bash
# 查看帮助
./build/exploration_3d -h

# 使用默认配置运行
./build/exploration_3d

# 指定配置文件
./build/exploration_3d -c config/exploration_config.yaml
```

## 配置说明

参考 `config/exploration_config.yaml` 文件，主要配置项：

| 参数 | 说明 |
|------|------|
| `map.resolution` | 体素分辨率 (米) |
| `map.max_range` | 最大感知范围 |
| `exploration.min_frontier_size` | 最小前沿点簇大小 |
| `exploration.goal_tolerance` | 目标到达容差 |
| `astar.max_iterations` | A* 最大迭代次数 |
| `mqtt.broker_address` | MQTT Broker 地址 |

## 算法说明

### 前沿点检测
- 使用 26 邻域检测前沿体素（已知空闲且相邻未知区域）
- DBSCAN 聚类算法合并相邻前沿点
- 评分函数考虑：信息增益、距离代价、方向一致性

### 路径规划
- 3D A* 搜索，支持 26 方向移动
- 对角线移动时检查角落穿墙
- Douglas-Peucker 算法简化路径

## License

MIT
