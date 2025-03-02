//
// Created by lbw on 25-3-2.
//

#include "environment/Environment.h"

Environment::Environment():rclcpp::Node("Environment")
{
    markers_pub_ = this->create_publisher<MarkerArray>("environment/markers", 10);
}

Environment::~Environment()
{
}
