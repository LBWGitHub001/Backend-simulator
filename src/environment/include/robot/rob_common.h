//
// Created by lbw on 25-3-2.
//

#ifndef ROB_COMMON_H
#define ROB_COMMON_H
#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <map>

namespace robot
{
  struct State
  {
    geometry_msgs::msg::TransformStamped robot_center_;
    float v_x, v_y, v_z, w;
    float r;
    Eigen::Matrix3d R;
  };

  struct Armor
  {
    bool visual{};
    visualization_msgs::msg::Marker armor;
  };

  struct RobotMarkers
  {
    std::string frame;
    geometry_msgs::msg::TransformStamped center_transform;
    visualization_msgs::msg::MarkerArray markers;
    std::vector<Armor> armors;
    visualization_msgs::msg::Marker center;
  };

}
#endif //COMMON_H
