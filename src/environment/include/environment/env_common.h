//
// Created by lbw on 25-3-2.
//

#ifndef ENV_COMMON_H
#define ENV_COMMON_H
#include <visualization_msgs/msg/marker.hpp>
namespace environment
{
    struct SiteParam
    {
        int width;
        int length;
    };

    struct VisualElements
    {
        visualization_msgs::msg::Marker sight;
        visualization_msgs::msg::Marker ground;
    };
}

#endif //ENV_COMMON_H
