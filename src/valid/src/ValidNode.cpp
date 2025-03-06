//
// Created by lbw on 25-3-6.
//

#include "ValidNode.h"

ValidNode::ValidNode(): rclcpp::Node("ValidNode")
{
    armors_sub_ = this->create_subscription<Armors>("environment/armors", 10,
                                                    std::bind(&ValidNode::armorsCallback, this, std::placeholders::_1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

ValidNode::~ValidNode()
{
}

void ValidNode::armorsCallback(const Armors::ConstSharedPtr& msg)
{
}
