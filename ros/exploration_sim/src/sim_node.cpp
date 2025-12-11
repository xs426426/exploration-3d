#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "mock_drone.hpp"
#include "octomap_manager.hpp"
#include "frontier_detector.hpp"
#include "path_planner.hpp"

using namespace exploration;

class ExplorationSimNode {
public:
    ExplorationSimNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), iteration_(0), running_(false) {

        // 参数
        pnh_.param<std::string>("pcd_file", pcdFile_, "zzxl3.pcd");
        pnh_.param<int>("max_iterations", maxIterations_, 100);
        pnh_.param<double>("resolution", resolution_, 0.1);
        pnh_.param<double>("sim_rate", simRate_, 2.0);  // 2 Hz 仿真速度

        // 发布者
        pubDronePose_ = nh_.advertise<geometry_msgs::PoseStamped>("/drone/pose", 10);
        pubTrajectory_ = nh_.advertise<nav_msgs::Path>("/drone/trajectory", 10);
        pubCurrentPath_ = nh_.advertise<nav_msgs::Path>("/drone/current_path", 10);
        pubOctomap_ = nh_.advertise<octomap_msgs::Octomap>("/octomap", 10);
        pubFrontiers_ = nh_.advertise<visualization_msgs::MarkerArray>("/frontiers", 10);
        pubEnvCloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/environment", 1, true);  // latched
        pubVisibleCloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/visible_cloud", 10);
        pubDroneMarker_ = nh_.advertise<visualization_msgs::Marker>("/drone/marker", 10);

        ROS_INFO("Exploration Sim Node initialized");
    }

    bool initialize() {
        ROS_INFO("Loading environment: %s", pcdFile_.c_str());

        // 配置传感器
        SensorConfig sensorCfg;
        sensorCfg.horizontalFov = 70.0;
        sensorCfg.verticalFov = 60.0;
        sensorCfg.maxRange = 4.0;
        sensorCfg.minRange = 0.2;
        sensorCfg.maxPointsPerFrame = 5000;
        drone_.setSensorConfig(sensorCfg);

        // 配置运动学
        DroneKinematicsConfig kinCfg;
        kinCfg.maxVelocity = 0.5;
        kinCfg.positionTolerance = 0.15;
        drone_.setKinematicsConfig(kinCfg);

        // 加载环境
        if (!drone_.loadEnvironment(pcdFile_)) {
            ROS_ERROR("Failed to load PCD file!");
            return false;
        }

        // 发布环境点云 (只发一次)
        publishEnvironmentCloud();

        // 设置初始位置
        Point3D minBound, maxBound;
        drone_.getEnvironmentBounds(minBound, maxBound);

        ROS_INFO("Environment bounds: min(%.2f, %.2f, %.2f) max(%.2f, %.2f, %.2f)",
                 minBound.x, minBound.y, minBound.z,
                 maxBound.x, maxBound.y, maxBound.z);

        // 起始位置在环境中心，Z轴设置在地面上方
        // 注意：点云实际Z值从约0开始，但边界可能有异常值
        Point3D startPos = {
            (minBound.x + maxBound.x) / 2.0,
            (minBound.y + maxBound.y) / 2.0,
            1.5  // 直接设为1.5米高度（点云地面约为Z=0）
        };
        drone_.setInitialPose(startPos, 0.0);

        ROS_INFO("Start position: (%.2f, %.2f, %.2f)", startPos.x, startPos.y, startPos.z);

        // 初始化 OctoMap
        octomap_ = std::make_unique<OctoMapManager>(resolution_);

        // 初始化探索组件
        ExplorationConfig config;
        config.resolution = resolution_;
        config.minClusterSize = 5;
        config.clusterRadius = 0.5;
        config.astarMaxIterations = 50000;
        config.safetyMargin = 0.2;

        frontierDetector_ = std::make_unique<FrontierDetector>(config);
        pathPlanner_ = std::make_unique<PathPlanner>(config);

        // 初始化轨迹
        trajectory_.header.frame_id = "world";

        running_ = true;
        return true;
    }

    void spin() {
        ros::Rate rate(simRate_);

        while (ros::ok() && running_ && iteration_ < maxIterations_) {
            ros::spinOnce();
            runOneIteration();
            rate.sleep();
        }

        ROS_INFO("Exploration finished! Iterations: %d", iteration_);
    }

private:
    void runOneIteration() {
        ROS_INFO("--- Iteration %d ---", iteration_ + 1);

        Point3D currentPos = drone_.getPosition();
        double currentYaw = drone_.getYaw();

        // 发布无人机位姿
        publishDronePose(currentPos, currentYaw);
        publishDroneMarker(currentPos, currentYaw);
        broadcastTF(currentPos, currentYaw);

        // 感知
        PointCloud visibleCloud = drone_.perceive();
        ROS_INFO("Perceived %zu points", visibleCloud.points.size());

        if (visibleCloud.points.empty()) {
            // 旋转无人机寻找点云 - 设置当前位置为目标以触发yaw更新
            double newYaw = currentYaw + M_PI / 4.0;  // 每次旋转45度
            Point3D currentPosForRotation = currentPos;
            currentPosForRotation.x += 0.01;  // 微小偏移触发移动
            drone_.setTargetPosition(currentPosForRotation);
            for (int i = 0; i < 10; i++) {
                drone_.update(0.1);  // 更新状态让yaw改变
            }
            ROS_WARN("No points perceived, rotating to find points (yaw=%.1f deg)",
                     drone_.getYaw() * 180.0 / M_PI);
            iteration_++;
            return;
        }

        // 发布可见点云
        publishVisibleCloud(visibleCloud);

        // 更新 OctoMap
        octomap_->insertPointCloud(visibleCloud, currentPos);
        publishOctomap();

        // 检测前沿点
        auto frontiers = frontierDetector_->detectFrontiers(*octomap_, currentPos);
        ROS_INFO("Detected %zu frontier clusters", frontiers.size());
        publishFrontiers(frontiers);

        if (frontiers.empty()) {
            ROS_INFO("Exploration complete! No more frontiers.");
            running_ = false;
            return;
        }

        // 选择最佳前沿点
        const auto& bestFrontier = frontiers[0];
        ROS_INFO("Target: (%.2f, %.2f, %.2f) score=%.2f",
                 bestFrontier.centroid.x, bestFrontier.centroid.y,
                 bestFrontier.centroid.z, bestFrontier.score);

        // 路径规划
        Path3D path = pathPlanner_->planPath(currentPos, bestFrontier.centroid, *octomap_);

        if (!path.isValid || path.waypoints.empty()) {
            ROS_WARN("Path planning failed, marking as unreachable");
            frontierDetector_->addUnreachableGoal(bestFrontier.centroid);
            iteration_++;
            return;
        }

        ROS_INFO("Planned path: %zu waypoints, length=%.2fm",
                 path.waypoints.size(), path.totalLength);

        // 发布当前规划路径
        publishCurrentPath(path);

        // 模拟飞行
        double simDt = 0.1;
        for (const auto& waypoint : path.waypoints) {
            drone_.setTargetPosition(waypoint);
            int simSteps = 0;
            while (!drone_.hasReachedTarget() && simSteps < 100) {
                drone_.update(simDt);
                simSteps++;

                // 更新可视化
                Point3D pos = drone_.getPosition();
                double yaw = drone_.getYaw();
                publishDronePose(pos, yaw);
                publishDroneMarker(pos, yaw);
                broadcastTF(pos, yaw);

                // 添加到轨迹
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = ros::Time::now();
                pose.header.frame_id = "world";
                pose.pose.position.x = pos.x;
                pose.pose.position.y = pos.y;
                pose.pose.position.z = pos.z;
                trajectory_.poses.push_back(pose);
            }
        }

        // 发布完整轨迹
        trajectory_.header.stamp = ros::Time::now();
        pubTrajectory_.publish(trajectory_);

        // 更新历史
        Point3D newPos = drone_.getPosition();
        frontierDetector_->addVisitedGoal(newPos);

        iteration_++;
    }

    void publishDronePose(const Point3D& pos, double yaw) {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";
        msg.pose.position.x = pos.x;
        msg.pose.position.y = pos.y;
        msg.pose.position.z = pos.z;
        msg.pose.orientation.w = std::cos(yaw / 2.0);
        msg.pose.orientation.z = std::sin(yaw / 2.0);
        pubDronePose_.publish(msg);
    }

    void publishDroneMarker(const Point3D& pos, double yaw) {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "world";
        marker.ns = "drone";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;

        // 如果没有无人机模型，用箭头代替
        marker.type = visualization_msgs::Marker::ARROW;
        marker.pose.position.x = pos.x;
        marker.pose.position.y = pos.y;
        marker.pose.position.z = pos.z;
        marker.pose.orientation.w = std::cos(yaw / 2.0);
        marker.pose.orientation.z = std::sin(yaw / 2.0);

        marker.scale.x = 0.5;  // 长度
        marker.scale.y = 0.1;  // 宽度
        marker.scale.z = 0.1;  // 高度

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        pubDroneMarker_.publish(marker);
    }

    void broadcastTF(const Point3D& pos, double yaw) {
        geometry_msgs::TransformStamped tf;
        tf.header.stamp = ros::Time::now();
        tf.header.frame_id = "world";
        tf.child_frame_id = "drone";
        tf.transform.translation.x = pos.x;
        tf.transform.translation.y = pos.y;
        tf.transform.translation.z = pos.z;
        tf.transform.rotation.w = std::cos(yaw / 2.0);
        tf.transform.rotation.z = std::sin(yaw / 2.0);
        tfBroadcaster_.sendTransform(tf);
    }

    void publishOctomap() {
        octomap_msgs::Octomap msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";

        octomap::OcTree* tree = octomap_->getOcTree();
        if (tree) {
            octomap_msgs::fullMapToMsg(*tree, msg);
            pubOctomap_.publish(msg);
        }
    }

    void publishFrontiers(const std::vector<FrontierCluster>& frontiers) {
        visualization_msgs::MarkerArray markers;

        // 清除旧的
        visualization_msgs::Marker deleteAll;
        deleteAll.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(deleteAll);

        int id = 0;
        for (const auto& frontier : frontiers) {
            // 前沿点簇中心
            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = "world";
            marker.ns = "frontier_centers";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = frontier.centroid.x;
            marker.pose.position.y = frontier.centroid.y;
            marker.pose.position.z = frontier.centroid.z;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.3;

            // 最佳前沿点用红色，其他用蓝色
            if (id == 1) {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            } else {
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
            }
            marker.color.a = 0.8;

            markers.markers.push_back(marker);
        }

        pubFrontiers_.publish(markers);
    }

    void publishCurrentPath(const Path3D& path) {
        nav_msgs::Path msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";

        for (const auto& wp : path.waypoints) {
            geometry_msgs::PoseStamped pose;
            pose.header = msg.header;
            pose.pose.position.x = wp.x;
            pose.pose.position.y = wp.y;
            pose.pose.position.z = wp.z;
            pose.pose.orientation.w = 1.0;
            msg.poses.push_back(pose);
        }

        pubCurrentPath_.publish(msg);
    }

    void publishEnvironmentCloud() {
        // 获取环境点云并发布
        pcl::PointCloud<pcl::PointXYZ> cloud;

        // 从 mock_drone 获取环境边界，采样一些点用于显示
        Point3D minB, maxB;
        drone_.getEnvironmentBounds(minB, maxB);

        // 这里简单发布边界框
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";
        pubEnvCloud_.publish(msg);
    }

    void publishVisibleCloud(const PointCloud& cloud) {
        pcl::PointCloud<pcl::PointXYZ> pclCloud;
        pclCloud.header.frame_id = "world";

        for (const auto& pt : cloud.points) {
            pclCloud.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
        }

        // 打印前几个点的坐标用于调试
        if (!cloud.points.empty()) {
            ROS_INFO("First point: (%.2f, %.2f, %.2f)",
                     cloud.points[0].x, cloud.points[0].y, cloud.points[0].z);
        }

        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(pclCloud, msg);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";

        // 确保消息有效
        msg.is_dense = true;
        msg.is_bigendian = false;

        pubVisibleCloud_.publish(msg);

        ROS_INFO("Published visible cloud: %zu points, width=%d, height=%d",
                 cloud.points.size(), msg.width, msg.height);
    }

private:
    ros::NodeHandle nh_, pnh_;

    // 参数
    std::string pcdFile_;
    int maxIterations_;
    double resolution_;
    double simRate_;

    // 发布者
    ros::Publisher pubDronePose_;
    ros::Publisher pubTrajectory_;
    ros::Publisher pubCurrentPath_;
    ros::Publisher pubOctomap_;
    ros::Publisher pubFrontiers_;
    ros::Publisher pubEnvCloud_;
    ros::Publisher pubVisibleCloud_;
    ros::Publisher pubDroneMarker_;

    // TF
    tf2_ros::TransformBroadcaster tfBroadcaster_;

    // 探索组件
    MockDrone drone_;
    std::unique_ptr<OctoMapManager> octomap_;
    std::unique_ptr<FrontierDetector> frontierDetector_;
    std::unique_ptr<PathPlanner> pathPlanner_;

    // 状态
    int iteration_;
    bool running_;
    nav_msgs::Path trajectory_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "exploration_sim_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ExplorationSimNode node(nh, pnh);

    if (!node.initialize()) {
        ROS_ERROR("Failed to initialize!");
        return 1;
    }

    node.spin();

    return 0;
}
