#include "mqtt_client.hpp"
#include "drone_message.pb.h"
#include <iostream>

namespace exploration {

// 连接回调
class ConnectCallback : public virtual mqtt::iaction_listener {
public:
    void on_failure(const mqtt::token& tok) override {
        std::cerr << "[MQTT] 连接失败" << std::endl;
    }

    void on_success(const mqtt::token& tok) override {
        std::cout << "[MQTT] 连接成功" << std::endl;
    }
};

// 消息回调
class MessageCallback : public virtual mqtt::callback {
public:
    using MessageHandler = std::function<void(const std::string&, const std::string&)>;

    explicit MessageCallback(MessageHandler handler) : handler_(handler) {}

    void message_arrived(mqtt::const_message_ptr msg) override {
        if (handler_) {
            handler_(msg->get_topic(), msg->get_payload_str());
        }
    }

    void connection_lost(const std::string& cause) override {
        std::cerr << "[MQTT] 连接断开: " << cause << std::endl;
    }

private:
    MessageHandler handler_;
};

MqttClient::MqttClient(const ExplorationConfig& config)
    : config_(config)
{
    client_ = std::make_unique<mqtt::async_client>(
        config_.mqttBroker,
        config_.mqttClientId
    );

    std::cout << "[MqttClient] 初始化完成, Broker=" << config_.mqttBroker << std::endl;
}

MqttClient::~MqttClient() {
    disconnect();
}

bool MqttClient::connect() {
    try {
        // 设置消息回调
        auto callback = std::make_shared<MessageCallback>(
            [this](const std::string& topic, const std::string& payload) {
                this->onMessage(topic, payload);
            }
        );
        client_->set_callback(*callback);

        // 连接选项
        mqtt::connect_options connOpts;
        connOpts.set_clean_session(true);
        connOpts.set_keep_alive_interval(20);
        connOpts.set_automatic_reconnect(true);

        std::cout << "[MQTT] 正在连接到 " << config_.mqttBroker << "..." << std::endl;

        mqtt::token_ptr tok = client_->connect(connOpts);
        tok->wait();

        // 订阅主题
        client_->subscribe(config_.mqttTopicPointCloud, 0)->wait();
        client_->subscribe(config_.mqttTopicOdometry, 0)->wait();

        std::cout << "[MQTT] 已订阅: " << config_.mqttTopicPointCloud << std::endl;
        std::cout << "[MQTT] 已订阅: " << config_.mqttTopicOdometry << std::endl;

        connected_ = true;
        return true;

    } catch (const mqtt::exception& e) {
        std::cerr << "[MQTT] 连接失败: " << e.what() << std::endl;
        return false;
    }
}

void MqttClient::disconnect() {
    if (connected_) {
        try {
            client_->disconnect()->wait();
            connected_ = false;
            std::cout << "[MQTT] 已断开连接" << std::endl;
        } catch (const mqtt::exception& e) {
            std::cerr << "[MQTT] 断开失败: " << e.what() << std::endl;
        }
    }
}

bool MqttClient::isConnected() const {
    return connected_ && client_->is_connected();
}

void MqttClient::setPointCloudCallback(PointCloudCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    pointCloudCallback_ = callback;
}

void MqttClient::setOdometryCallback(OdometryCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    odometryCallback_ = callback;
}

bool MqttClient::publishMission(const Mission& mission) {
    if (!isConnected()) {
        std::cerr << "[MQTT] 未连接，无法发布任务" << std::endl;
        return false;
    }

    try {
        std::string payload = serializeMission(mission);
        mqtt::message_ptr msg = mqtt::make_message(config_.mqttTopicMission, payload);
        msg->set_qos(1);
        client_->publish(msg)->wait();

        std::cout << "[MQTT] 任务已发布: " << mission.id
                  << ", 路点数=" << mission.waypoints.size() << std::endl;
        return true;

    } catch (const mqtt::exception& e) {
        std::cerr << "[MQTT] 发布任务失败: " << e.what() << std::endl;
        return false;
    }
}

bool MqttClient::publishExecution(const std::string& missionId, int action) {
    if (!isConnected()) {
        std::cerr << "[MQTT] 未连接，无法发布执行指令" << std::endl;
        return false;
    }

    try {
        std::string payload = serializeExecution(missionId, action);
        mqtt::message_ptr msg = mqtt::make_message(config_.mqttTopicExecution, payload);
        msg->set_qos(1);
        client_->publish(msg)->wait();

        const char* actionNames[] = {"START", "PAUSE", "RESUME", "STOP"};
        std::cout << "[MQTT] 执行指令已发布: " << missionId
                  << ", action=" << actionNames[action] << std::endl;
        return true;

    } catch (const mqtt::exception& e) {
        std::cerr << "[MQTT] 发布执行指令失败: " << e.what() << std::endl;
        return false;
    }
}

void MqttClient::onMessage(const std::string& topic, const std::string& payload) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (topic == config_.mqttTopicPointCloud && pointCloudCallback_) {
        PointCloud cloud = parsePointCloud(payload);
        pointCloudCallback_(cloud);
    }
    else if (topic == config_.mqttTopicOdometry && odometryCallback_) {
        Pose pose = parseOdometry(payload);
        odometryCallback_(pose);
    }
}

PointCloud MqttClient::parsePointCloud(const std::string& data) {
    PointCloud result;

    daf::PointCloud pbCloud;
    if (!pbCloud.ParseFromString(data)) {
        std::cerr << "[MQTT] 点云解析失败" << std::endl;
        return result;
    }

    result.timestamp = pbCloud.timestamp();
    result.points.reserve(pbCloud.points_size());

    for (int i = 0; i < pbCloud.points_size(); ++i) {
        const auto& p = pbCloud.points(i);
        result.points.emplace_back(p.x(), p.y(), p.z());
    }

    return result;
}

Pose MqttClient::parseOdometry(const std::string& data) {
    Pose result;

    daf::Odometry pbOdom;
    if (!pbOdom.ParseFromString(data)) {
        std::cerr << "[MQTT] 里程计解析失败" << std::endl;
        return result;
    }

    if (pbOdom.has_pose()) {
        const auto& pose = pbOdom.pose();
        if (pose.has_position()) {
            result.position.x = pose.position().x();
            result.position.y = pose.position().y();
            result.position.z = pose.position().z();
        }
        if (pose.has_orientation()) {
            result.qw = pose.orientation().w();
            result.qx = pose.orientation().x();
            result.qy = pose.orientation().y();
            result.qz = pose.orientation().z();

            // 四元数转yaw
            double siny_cosp = 2.0 * (result.qw * result.qz + result.qx * result.qy);
            double cosy_cosp = 1.0 - 2.0 * (result.qy * result.qy + result.qz * result.qz);
            result.yaw = std::atan2(siny_cosp, cosy_cosp);
        }
    }

    return result;
}

std::string MqttClient::serializeMission(const Mission& mission) {
    daf::Mission pbMission;
    pbMission.set_id(mission.id);

    for (const auto& wp : mission.waypoints) {
        daf::Task* task = pbMission.add_tasks();
        daf::AutoPilot* autoPilot = task->mutable_auto_pilot();

        daf::Position* pos = autoPilot->mutable_position();
        pos->set_x(wp.x);
        pos->set_y(wp.y);
        pos->set_z(wp.z);

        autoPilot->set_yaw(mission.yaw);

        daf::CameraParam* cam = autoPilot->mutable_camera_param();
        cam->set_on(false);
        cam->set_mode(0);
        cam->set_interval(0);
    }

    return pbMission.SerializeAsString();
}

std::string MqttClient::serializeExecution(const std::string& missionId, int action) {
    daf::Execution pbExec;
    pbExec.set_id(missionId);
    pbExec.set_action(action);

    return pbExec.SerializeAsString();
}

}  // namespace exploration
