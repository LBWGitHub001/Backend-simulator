//
// Created by lbw on 25-3-2.
//
#include "ValidNode.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ValidNode>());
    rclcpp::shutdown();
}