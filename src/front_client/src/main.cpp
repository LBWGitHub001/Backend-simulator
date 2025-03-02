//
// Created by lbw on 25-1-15.
//
#include <rclcpp/rclcpp.hpp>
#include "Robot.h"
int main (int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot>());
    rclcpp::shutdown();
}