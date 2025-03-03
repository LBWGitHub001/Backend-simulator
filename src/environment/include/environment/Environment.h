//
// Created by lbw on 25-3-2.
//

#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H
//ros
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
//project
#include "environment/env_common.h"
#include "robot/robot.h"
#include "interfaces/msg/armor.hpp"
#include "interfaces/msg/armors.hpp"
#include "interfaces/msg/label.hpp"

class Environment :
    public rclcpp::Node
{
public:
    using Marker = visualization_msgs::msg::Marker;
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using Transform = geometry_msgs::msg::Transform;
    using TransformStamped = geometry_msgs::msg::TransformStamped;
    using SiteParam = environment::SiteParam;
    using VisualElements = environment::VisualElements;
    using Armor = interfaces::msg::Armor;
    using Armors = interfaces::msg::Armors;


    Environment();
    ~Environment();

    void init();
    void initMarkers();
    std::unique_ptr<Robot> initRobot(const std::string& type,
                                     float x, float y, float z, float r,
                                     float vx, float vy, float vz, float w);

private:
    //visual
    rclcpp::Publisher<MarkerArray>::SharedPtr self_markers_pub_;
    rclcpp::Publisher<MarkerArray>::SharedPtr robot_markers_pub_;
    rclcpp::Publisher<MarkerArray>::SharedPtr next_robot_markers_pub_;
    rclcpp::TimerBase::SharedPtr marker_timer_;
    void publish_markers();

    //tf
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};
    std::unique_ptr<tf2_ros::TransformListener> listener_{nullptr};
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_to_self_broadcast_{nullptr};
    std::unique_ptr<tf2_ros::TransformBroadcaster> self_to_robot_broadcast_{nullptr};

    //Site
    SiteParam site_param_;
    VisualElements visual_elements_;

    //state
    std::vector<std::unique_ptr<Robot>> robot_;
    std::shared_ptr<Robot::DirVector> sight_dir_;
    TransformStamped odom_to_self;

    //pub
    rclcpp::Publisher<Armors>::SharedPtr armors_pub_;
};


#endif //ENVIRONMENT_H
