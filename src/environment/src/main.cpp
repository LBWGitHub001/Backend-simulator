//
// Created by lbw on 25-3-2.
//
#include "environment/Environment.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Environment>());
    rclcpp::shutdown();
}