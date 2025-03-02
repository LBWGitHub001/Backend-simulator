//
// Created by lbw on 25-3-2.
//

#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H
//ros
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
//project
#include "robot/robot.h"

class Environment :
    public rclcpp::Node
{
public:

  using MarkerArray = visualization_msgs::msg::Marker;

    Environment();
    ~Environment();

    void init();

private:
    //visual
    rclcpp::Publisher<MarkerArray>::SharedPtr markers_pub_;

    //state
    std::vector<std::unique_ptr<Robot>> robot_;
};


#endif //ENVIRONMENT_H
