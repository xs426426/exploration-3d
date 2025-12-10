#include "exploration_engine.hpp"
#include "config_loader.hpp"
#include <iostream>
#include <csignal>
#include <atomic>

using namespace exploration;

// 全局引擎指针（用于信号处理）
std::atomic<ExplorationEngine*> g_engine{nullptr};

// 信号处理
void signalHandler(int signal) {
    std::cout << "\n[Main] 收到信号 " << signal << ", 正在停止..." << std::endl;
    if (g_engine) {
        g_engine.load()->stop();
    }
}

void printUsage(const char* programName) {
    std::cout << "用法: " << programName << " [选项]\n"
              << "\n"
              << "选项:\n"
              << "  -c, --config <file>   指定配置文件路径 (默认: config/exploration_config.yaml)\n"
              << "  -m, --mode <mode>     路径规划模式: direct 或 astar (默认: astar)\n"
              << "  -b, --broker <url>    MQTT broker地址 (默认: tcp://localhost:1883)\n"
              << "  -r, --resolution <m>  地图分辨率 (默认: 0.1)\n"
              << "  -g, --generate        生成默认配置文件并退出\n"
              << "  -h, --help            显示此帮助信息\n"
              << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "========================================" << std::endl;
    std::cout << "   3D 自主探索系统 (C++ 版本)" << std::endl;
    std::cout << "   基于 OctoMap + A* 路径规划" << std::endl;
    std::cout << "========================================\n" << std::endl;

    // 默认参数
    std::string configFile = "config/exploration_config.yaml";
    std::string pathMode = "";
    std::string mqttBroker = "";
    double resolution = -1;
    bool generateConfig = false;

    // 解析命令行参数
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        }
        else if (arg == "-g" || arg == "--generate") {
            generateConfig = true;
        }
        else if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
            configFile = argv[++i];
        }
        else if ((arg == "-m" || arg == "--mode") && i + 1 < argc) {
            pathMode = argv[++i];
        }
        else if ((arg == "-b" || arg == "--broker") && i + 1 < argc) {
            mqttBroker = argv[++i];
        }
        else if ((arg == "-r" || arg == "--resolution") && i + 1 < argc) {
            resolution = std::stod(argv[++i]);
        }
        else {
            std::cerr << "未知参数: " << arg << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }

    // 生成默认配置文件
    if (generateConfig) {
        ExplorationConfig defaultConfig = ConfigLoader::getDefaultConfig();
        if (ConfigLoader::saveToYaml(defaultConfig, configFile)) {
            std::cout << "默认配置已生成: " << configFile << std::endl;
            return 0;
        } else {
            return 1;
        }
    }

    // 加载配置
    ExplorationConfig config = ConfigLoader::loadFromYaml(configFile);

    // 命令行参数覆盖配置文件
    if (!pathMode.empty()) {
        config.pathPlanningMode = pathMode;
    }
    if (!mqttBroker.empty()) {
        config.mqttBroker = mqttBroker;
    }
    if (resolution > 0) {
        config.resolution = resolution;
    }

    // 打印配置
    ConfigLoader::printConfig(config);

    // 注册信号处理
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // 创建探索引擎
    ExplorationEngine engine(config);
    g_engine = &engine;

    // 初始化
    if (!engine.initialize()) {
        std::cerr << "[Main] 初始化失败" << std::endl;
        return 1;
    }

    // 启动探索
    if (!engine.start()) {
        std::cerr << "[Main] 启动失败" << std::endl;
        return 1;
    }

    // 运行主循环
    engine.run();

    // 输出最终统计
    auto stats = engine.getStatistics();
    std::cout << "\n[Main] 探索结束" << std::endl;
    std::cout << "  已探索体积: " << stats.exploredVolume << " m³" << std::endl;
    std::cout << "  访问路点数: " << stats.waypointsVisited << std::endl;
    std::cout << "  总飞行距离: " << stats.totalDistance << " m" << std::endl;
    std::cout << "  运行时间: " << stats.elapsedSeconds << " s" << std::endl;

    g_engine = nullptr;
    return 0;
}
