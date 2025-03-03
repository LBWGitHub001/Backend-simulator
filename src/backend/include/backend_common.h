//
// Created by lbw on 25-3-3.
//

#ifndef BACKEND_COMMON_H
#define BACKEND_COMMON_H
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace backend{
struct Preset
{
    visualization_msgs::msg::Marker center_esl;
    visualization_msgs::msg::Marker armor;
};


}
#endif //BACKEND_COMMON_H
