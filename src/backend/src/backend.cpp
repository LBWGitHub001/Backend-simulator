//
// Created by lbw on 25-3-2.
//

#include "backend.h"

BackEnd::BackEnd():rclcpp::Node("backend")
{
    RCLCPP_INFO(this->get_logger(), "BackEndNode is Started!");
    armors_sub_ = this->create_subscription<Armors>("environment/armors",10,
        std::bind(&BackEnd::armors_callback, this, std::placeholders::_1));
    predict_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&BackEnd::predict_timer_callback, this));


}

BackEnd::~BackEnd() = default;

void BackEnd::armors_callback(const Armors::SharedPtr msg)
{
    armors_ = *msg;
}

void BackEnd::predict_timer_callback()
{

}



