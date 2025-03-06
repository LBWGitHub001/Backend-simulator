//
// Created by lbw on 25-3-6.
//

#ifndef VALIDNODE_H
#define VALIDNODE_H
#include <rclcpp/rclcpp.hpp>
#include <AsyncInferFrame.hpp>
#include <Eigen/Dense>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "interfaces/msg/armors.hpp"
#include "interfaces/msg/armor.hpp"
class ValidNode : public rclcpp::Node
{
public:
  using Armor = interfaces::msg::Armor;
  using Armors = interfaces::msg::Armors;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;


  ValidNode();
  ~ValidNode() override;

private:
  //sub to armors
  rclcpp::Subscription<Armors>::SharedPtr armors_sub_;
  void armorsCallback(const Armors::ConstSharedPtr& msg);
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


  //visual
  rclcpp::Publisher<MarkerArray>::SharedPtr markers_pub_;

  //inferer
  std::unique_ptr<AUTO_INFER> infer_{nullptr};


};


#endif //VALIDNODE_H
