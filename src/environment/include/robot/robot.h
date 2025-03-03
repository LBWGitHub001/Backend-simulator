//
// Created by lbw on 25-3-2.
//

#ifndef ROBOT_H
#define ROBOT_H
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "robot/rob_common.h"

class Robot
{
public:
    using DirVector = Eigen::Vector3d;
    using TransformStamp = geometry_msgs::msg::TransformStamped;
    using State = robot::State;
    using RobotMarkers = robot::RobotMarkers;
    using Marker = visualization_msgs::msg::Marker;

struct Preset
{
    Marker center;
    Marker visible_armor;
    Marker invisible_armor;
};

    Robot();
    explicit Robot(const std::string& frame);
    ~Robot();

    void setSight(std::shared_ptr<DirVector> sight);
    void setTimestamp(const rclcpp::Time& time);
    void setParent(const std::string& frame);
    void setCenter(float x, float y, float z);
    void setVelocity(float vx, float vy, float vz);
    void setW(float w);
    void setR(float r);
    void start() const;

    RobotMarkers getState(const rclcpp::Time& time);

private:
    int set_complete_;

    std::string frame_;
    State robot_state_;
    Marker armor_standard_;
    std::shared_ptr<DirVector> sight_dir_;

    //preset
    Preset preset_;
    Eigen::Matrix3d armor_R_;
    void initMarkers();
};


#endif //ROBOT_H
