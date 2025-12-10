#pragma once

#include "types.hpp"
#include <mqtt/async_client.h>
#include <functional>
#include <memory>
#include <mutex>
#include <atomic>
#include <thread>

namespace exploration {

/**
 * MQTT 客户端封装
 * 处理与无人机/Broker的通信
 */
class MqttClient {
public:
    explicit MqttClient(const ExplorationConfig& config);
    ~MqttClient();

    // 禁止拷贝
    MqttClient(const MqttClient&) = delete;
    MqttClient& operator=(const MqttClient&) = delete;

    /**
     * 连接到Broker
     */
    bool connect();

    /**
     * 断开连接
     */
    void disconnect();

    /**
     * 是否已连接
     */
    bool isConnected() const;

    /**
     * 设置回调
     */
    void setPointCloudCallback(PointCloudCallback callback);
    void setOdometryCallback(OdometryCallback callback);

    /**
     * 发布任务
     */
    bool publishMission(const Mission& mission);

    /**
     * 发布任务执行指令
     * @param action 0=START, 1=PAUSE, 2=RESUME, 3=STOP
     */
    bool publishExecution(const std::string& missionId, int action);

private:
    ExplorationConfig config_;
    std::unique_ptr<mqtt::async_client> client_;

    PointCloudCallback pointCloudCallback_;
    OdometryCallback odometryCallback_;

    std::atomic<bool> connected_{false};
    std::mutex mutex_;

    // 消息处理
    void onMessage(const std::string& topic, const std::string& payload);

    // Protobuf解析
    PointCloud parsePointCloud(const std::string& data);
    Pose parseOdometry(const std::string& data);

    // Protobuf序列化
    std::string serializeMission(const Mission& mission);
    std::string serializeExecution(const std::string& missionId, int action);
};

}  // namespace exploration
