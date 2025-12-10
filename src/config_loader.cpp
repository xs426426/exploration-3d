#include "config_loader.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

namespace exploration {

ExplorationConfig ConfigLoader::loadFromYaml(const std::string& filename) {
    ExplorationConfig config = getDefaultConfig();

    try {
        YAML::Node root = YAML::LoadFile(filename);

        // 地图参数
        if (root["map"]) {
            auto map = root["map"];
            if (map["resolution"]) config.resolution = map["resolution"].as<double>();
            if (map["min_height"]) config.minHeight = map["min_height"].as<double>();
            if (map["max_height"]) config.maxHeight = map["max_height"].as<double>();
        }

        // 探索参数
        if (root["exploration"]) {
            auto exp = root["exploration"];
            if (exp["max_distance"]) config.maxExplorationDistance = exp["max_distance"].as<double>();
            if (exp["goal_tolerance"]) config.goalTolerance = exp["goal_tolerance"].as<double>();
            if (exp["goal_timeout"]) config.goalTimeout = exp["goal_timeout"].as<double>();
            if (exp["update_rate"]) config.updateRate = exp["update_rate"].as<double>();
            if (exp["path_planning_mode"]) config.pathPlanningMode = exp["path_planning_mode"].as<std::string>();
        }

        // 前沿点检测参数
        if (root["frontier"]) {
            auto frontier = root["frontier"];
            if (frontier["cluster_radius"]) config.clusterRadius = frontier["cluster_radius"].as<double>();
            if (frontier["min_cluster_size"]) config.minClusterSize = frontier["min_cluster_size"].as<int>();
        }

        // 评分权重
        if (root["scoring"]) {
            auto scoring = root["scoring"];
            if (scoring["weight_info_gain"]) config.weightInfoGain = scoring["weight_info_gain"].as<double>();
            if (scoring["weight_distance"]) config.weightDistance = scoring["weight_distance"].as<double>();
            if (scoring["weight_consistency"]) config.weightConsistency = scoring["weight_consistency"].as<double>();
        }

        // A*参数
        if (root["astar"]) {
            auto astar = root["astar"];
            if (astar["heuristic_weight"]) config.astarHeuristicWeight = astar["heuristic_weight"].as<double>();
            if (astar["max_iterations"]) config.astarMaxIterations = astar["max_iterations"].as<int>();
            if (astar["simplify_tolerance"]) config.astarSimplifyTolerance = astar["simplify_tolerance"].as<double>();
        }

        // MQTT配置
        if (root["mqtt"]) {
            auto mqtt = root["mqtt"];
            if (mqtt["broker"]) config.mqttBroker = mqtt["broker"].as<std::string>();
            if (mqtt["client_id"]) config.mqttClientId = mqtt["client_id"].as<std::string>();
            if (mqtt["topic_point_cloud"]) config.mqttTopicPointCloud = mqtt["topic_point_cloud"].as<std::string>();
            if (mqtt["topic_odometry"]) config.mqttTopicOdometry = mqtt["topic_odometry"].as<std::string>();
            if (mqtt["topic_mission"]) config.mqttTopicMission = mqtt["topic_mission"].as<std::string>();
            if (mqtt["topic_execution"]) config.mqttTopicExecution = mqtt["topic_execution"].as<std::string>();
        }

        // 安全参数
        if (root["safety"]) {
            auto safety = root["safety"];
            if (safety["margin"]) config.safetyMargin = safety["margin"].as<double>();
        }

        std::cout << "[ConfigLoader] 配置已加载: " << filename << std::endl;

    } catch (const YAML::Exception& e) {
        std::cerr << "[ConfigLoader] 加载配置失败: " << e.what() << std::endl;
        std::cerr << "[ConfigLoader] 使用默认配置" << std::endl;
    }

    validateConfig(config);
    return config;
}

ExplorationConfig ConfigLoader::getDefaultConfig() {
    ExplorationConfig config;

    // 地图参数
    config.resolution = 0.1;
    config.minHeight = 0.3;
    config.maxHeight = 2.5;

    // 探索参数
    config.maxExplorationDistance = 10.0;
    config.goalTolerance = 0.5;
    config.goalTimeout = 60.0;
    config.updateRate = 10.0;
    config.pathPlanningMode = "astar";

    // 前沿点检测参数
    config.clusterRadius = 0.5;
    config.minClusterSize = 5;

    // 评分权重
    config.weightInfoGain = 1.0;
    config.weightDistance = 0.5;
    config.weightConsistency = 0.3;

    // A*参数
    config.astarHeuristicWeight = 1.0;
    config.astarMaxIterations = 100000;
    config.astarSimplifyTolerance = 0.2;

    // MQTT配置
    config.mqttBroker = "tcp://localhost:1883";
    config.mqttClientId = "exploration_3d";
    config.mqttTopicPointCloud = "drone/point_cloud";
    config.mqttTopicOdometry = "drone/odometry";
    config.mqttTopicMission = "drone/mission";
    config.mqttTopicExecution = "drone/execution";

    // 安全参数
    config.safetyMargin = 0.3;

    return config;
}

bool ConfigLoader::saveToYaml(const ExplorationConfig& config, const std::string& filename) {
    try {
        YAML::Emitter out;
        out << YAML::BeginMap;

        // 地图参数
        out << YAML::Key << "map" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "resolution" << YAML::Value << config.resolution;
        out << YAML::Key << "min_height" << YAML::Value << config.minHeight;
        out << YAML::Key << "max_height" << YAML::Value << config.maxHeight;
        out << YAML::EndMap;

        // 探索参数
        out << YAML::Key << "exploration" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "max_distance" << YAML::Value << config.maxExplorationDistance;
        out << YAML::Key << "goal_tolerance" << YAML::Value << config.goalTolerance;
        out << YAML::Key << "goal_timeout" << YAML::Value << config.goalTimeout;
        out << YAML::Key << "update_rate" << YAML::Value << config.updateRate;
        out << YAML::Key << "path_planning_mode" << YAML::Value << config.pathPlanningMode;
        out << YAML::EndMap;

        // 前沿点检测参数
        out << YAML::Key << "frontier" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "cluster_radius" << YAML::Value << config.clusterRadius;
        out << YAML::Key << "min_cluster_size" << YAML::Value << config.minClusterSize;
        out << YAML::EndMap;

        // 评分权重
        out << YAML::Key << "scoring" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "weight_info_gain" << YAML::Value << config.weightInfoGain;
        out << YAML::Key << "weight_distance" << YAML::Value << config.weightDistance;
        out << YAML::Key << "weight_consistency" << YAML::Value << config.weightConsistency;
        out << YAML::EndMap;

        // A*参数
        out << YAML::Key << "astar" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "heuristic_weight" << YAML::Value << config.astarHeuristicWeight;
        out << YAML::Key << "max_iterations" << YAML::Value << config.astarMaxIterations;
        out << YAML::Key << "simplify_tolerance" << YAML::Value << config.astarSimplifyTolerance;
        out << YAML::EndMap;

        // MQTT配置
        out << YAML::Key << "mqtt" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "broker" << YAML::Value << config.mqttBroker;
        out << YAML::Key << "client_id" << YAML::Value << config.mqttClientId;
        out << YAML::Key << "topic_point_cloud" << YAML::Value << config.mqttTopicPointCloud;
        out << YAML::Key << "topic_odometry" << YAML::Value << config.mqttTopicOdometry;
        out << YAML::Key << "topic_mission" << YAML::Value << config.mqttTopicMission;
        out << YAML::Key << "topic_execution" << YAML::Value << config.mqttTopicExecution;
        out << YAML::EndMap;

        // 安全参数
        out << YAML::Key << "safety" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "margin" << YAML::Value << config.safetyMargin;
        out << YAML::EndMap;

        out << YAML::EndMap;

        std::ofstream fout(filename);
        fout << out.c_str();
        fout.close();

        std::cout << "[ConfigLoader] 配置已保存: " << filename << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[ConfigLoader] 保存配置失败: " << e.what() << std::endl;
        return false;
    }
}

void ConfigLoader::printConfig(const ExplorationConfig& config) {
    std::cout << "\n========== 配置参数 ==========" << std::endl;

    std::cout << "[地图]" << std::endl;
    std::cout << "  分辨率: " << config.resolution << " m" << std::endl;
    std::cout << "  高度范围: [" << config.minHeight << ", " << config.maxHeight << "] m" << std::endl;

    std::cout << "[探索]" << std::endl;
    std::cout << "  最大探索距离: " << config.maxExplorationDistance << " m" << std::endl;
    std::cout << "  目标容差: " << config.goalTolerance << " m" << std::endl;
    std::cout << "  目标超时: " << config.goalTimeout << " s" << std::endl;
    std::cout << "  更新频率: " << config.updateRate << " Hz" << std::endl;
    std::cout << "  路径规划模式: " << config.pathPlanningMode << std::endl;

    std::cout << "[前沿点]" << std::endl;
    std::cout << "  聚类半径: " << config.clusterRadius << " m" << std::endl;
    std::cout << "  最小簇大小: " << config.minClusterSize << std::endl;

    std::cout << "[评分权重]" << std::endl;
    std::cout << "  信息增益: " << config.weightInfoGain << std::endl;
    std::cout << "  距离: " << config.weightDistance << std::endl;
    std::cout << "  一致性: " << config.weightConsistency << std::endl;

    std::cout << "[A*算法]" << std::endl;
    std::cout << "  启发式权重: " << config.astarHeuristicWeight << std::endl;
    std::cout << "  最大迭代: " << config.astarMaxIterations << std::endl;
    std::cout << "  简化容差: " << config.astarSimplifyTolerance << " m" << std::endl;

    std::cout << "[MQTT]" << std::endl;
    std::cout << "  Broker: " << config.mqttBroker << std::endl;
    std::cout << "  Client ID: " << config.mqttClientId << std::endl;

    std::cout << "[安全]" << std::endl;
    std::cout << "  安全边距: " << config.safetyMargin << " m" << std::endl;

    std::cout << "==============================\n" << std::endl;
}

bool ConfigLoader::validateConfig(ExplorationConfig& config) {
    bool valid = true;

    // 分辨率
    if (config.resolution <= 0) {
        std::cerr << "[ConfigLoader] 警告: 分辨率无效，使用默认值 0.1" << std::endl;
        config.resolution = 0.1;
        valid = false;
    }

    // 高度范围
    if (config.minHeight >= config.maxHeight) {
        std::cerr << "[ConfigLoader] 警告: 高度范围无效，使用默认值" << std::endl;
        config.minHeight = 0.3;
        config.maxHeight = 2.5;
        valid = false;
    }

    // 探索距离
    if (config.maxExplorationDistance <= 0) {
        std::cerr << "[ConfigLoader] 警告: 探索距离无效，使用默认值 10.0" << std::endl;
        config.maxExplorationDistance = 10.0;
        valid = false;
    }

    // 路径模式
    if (config.pathPlanningMode != "direct" && config.pathPlanningMode != "astar") {
        std::cerr << "[ConfigLoader] 警告: 路径模式无效，使用默认值 astar" << std::endl;
        config.pathPlanningMode = "astar";
        valid = false;
    }

    return valid;
}

}  // namespace exploration
