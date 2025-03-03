//
// Created by lbw on 25-3-2.
//

#ifndef BACKEND_H
#define BACKEND_H
//ros
#include <rclcpp/rclcpp.hpp>
//project
#include <interfaces/msg/armors.hpp>
#include <interfaces/msg/armor.hpp>

class BackEnd : public rclcpp::Node
{
public:

    using Armor = interfaces::msg::Armor;
    using Armors = interfaces::msg::Armors;

    BackEnd();
    ~BackEnd();

private:
    //sub to Armors
    rclcpp::Subscription<Armors>::SharedPtr armors_sub_;
    void armors_callback(const Armors::SharedPtr msg);
    Armors armors_;

    //predict
    rclcpp::TimerBase::SharedPtr predict_timer_;
    void predict_timer_callback();
};


#endif //BACKEND_H
