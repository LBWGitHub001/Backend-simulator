//
// Created by lbw on 25-3-2.
//

#ifndef BACKEND_H
#define BACKEND_H
//std
#include <Eigen/Dense>
//ros
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
//project
#include <interfaces/msg/armors.hpp>
#include <interfaces/msg/armor.hpp>
#include "backend_common.h"
#include "nnTrain/nn_MP.h"
#include "nnTrain/memory.h"
#include "logger/logger.h"
#include "nnTrain/writer.h"

class BackEnd : public rclcpp::Node
{
public:

    using Armor = interfaces::msg::Armor;
    using Armors = interfaces::msg::Armors;
    using TransformStamp = geometry_msgs::msg::TransformStamped;
    using Marker = visualization_msgs::msg::Marker;
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using String = std_msgs::msg::String;

    BackEnd();
    ~BackEnd();

    void init();
    void initMarkers();
    void initMemory();
private:
    //sub to Armors
    rclcpp::Subscription<Armors>::SharedPtr armors_sub_;
    void armors_callback(const Armors::SharedPtr msg);
    Armors armors_;
    rclcpp::Subscription<String>::SharedPtr change_state_sub_;
    void change_state_callback(const String::ConstSharedPtr msg);

    //predict
    rclcpp::TimerBase::SharedPtr predict_timer_;
    void predict_timer_callback();

    //visual
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> self_listener_;
    backend::Preset preset_;
    rclcpp::Publisher<MarkerArray>::SharedPtr esl_markers_pub_;

    //data for NN
    std::unique_ptr<Memory> memory_{nullptr};
};


#endif //BACKEND_H
