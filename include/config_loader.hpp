#pragma once

#include "types.hpp"
#include <string>

namespace exploration {

/**
 * 配置文件加载器
 * 支持YAML格式配置文件
 */
class ConfigLoader {
public:
    /**
     * 从YAML文件加载配置
     * @param filename 配置文件路径
     * @return 配置对象
     */
    static ExplorationConfig loadFromYaml(const std::string& filename);

    /**
     * 获取默认配置
     */
    static ExplorationConfig getDefaultConfig();

    /**
     * 保存配置到YAML文件
     */
    static bool saveToYaml(const ExplorationConfig& config, const std::string& filename);

    /**
     * 打印配置信息
     */
    static void printConfig(const ExplorationConfig& config);

private:
    /**
     * 验证配置有效性
     */
    static bool validateConfig(ExplorationConfig& config);
};

}  // namespace exploration
