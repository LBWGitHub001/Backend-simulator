//
// Created by lbw on 25-1-16.
//

#ifndef ROBOT_H
#define ROBOT_H
//std
#include <vector>
#include <random>
#include <Eigen/Dense>
//ros
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "interfaces/msg/armors.hpp"
#include "interfaces/msg/robot.hpp"
#include "interfaces/msg/direction.hpp"
#include "interfaces/srv/update_writer.hpp"

const std::string tf_camera = "camera";
const std::string tf_robot_center = "center";

class Robot :
    public rclcpp::Node
{
    using RobotInfo = interfaces::msg::Robot;
    using Armor = interfaces::msg::Armor;
    using Armors = interfaces::msg::Armors;
    using Marker = visualization_msgs::msg::Marker;
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using Point = geometry_msgs::msg::Point;
    using Direction = interfaces::msg::Direction;
    using TransfromStamped = geometry_msgs::msg::TransformStamped;
    using UpdateWriter = interfaces::srv::UpdateWriter;

public:
    Robot();
    ~Robot() = default;

private:
    //发布节点
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();
    rclcpp::Publisher<RobotInfo>::SharedPtr robot_pub_;
    rclcpp::Publisher<Armors>::SharedPtr armor_pub_;
    //坐标系发布
    std::unique_ptr<tf2_ros::TransformBroadcaster> center_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> camera_broadcaster_;
    //坐标系监听
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    //Markers
    rclcpp::Time timestamp_;
    rclcpp::Publisher<MarkerArray>::SharedPtr markers_pub_;
    Marker armor_marker_;
    Marker center_marker_;
    Marker robot_direction_marker_;
    void initMarkers();

    //系统初始化
    void initRobot();
    void publishMarkers();
    void publishRobotInfo();
    void camera_to_robot_direction();

    //工具函数
    static tf2::Quaternion vector_to_quaternion(const geometry_msgs::msg::Vector3& v);

    //参数
    double height_;
    Point center_;
    double yaw_;
    double v_yaw_;
    double r_;

    //系统值
    Direction camera_to_robot_;

    //更新机器人位置
    rclcpp::TimerBase::SharedPtr update_timer_;
    void update_timer_callback();
};


#endif //ROBOT_H
