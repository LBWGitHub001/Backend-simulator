//
// Created by lbw on 25-3-2.
//
#include "backend.h"
#include "nnTrain/writer.h"
int main(int argc, char **argv)
{
    Writer writer("/home/lbw/RM2025/Backend-simulator/src/backend/outputModels");
    std::this_thread::sleep_for(std::chrono::milliseconds(100000));
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BackEnd>());
    rclcpp::shutdown();
}