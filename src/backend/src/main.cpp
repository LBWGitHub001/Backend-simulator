//
// Created by lbw on 25-3-2.
//
#include "backend.h"
#include "nnTrain/writer.h"
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BackEnd>());
    rclcpp::shutdown();
}