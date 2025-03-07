//
// Created by lbw on 25-3-2.
//

#include <utility>

#include "robot/robot.h"

Robot::Robot()
{
}

Robot::Robot(const std::string& frame)
{
    set_complete_ = 0;
    frame_ = frame;
    robot_state_.robot_center_.child_frame_id = frame;
    robot_state_.R = Eigen::Matrix3d::Identity();
    double pitch = -0.3;
    armor_R_ << cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch);
    initMarkers();
}

Robot::~Robot()= default;

void Robot::setRobotType(const std::string& type)
{
    robot_type_ = type;
    set_complete_++;
}

void Robot::setSight(std::shared_ptr<DirVector> sight)
{
    sight_dir_ = sight;
    set_complete_++;
}

void Robot::setTimestamp(const rclcpp::Time& time)
{
    robot_state_.robot_center_.header.stamp = time;
    set_complete_++;
}

void Robot::setParent(const std::string& frame)
{
    robot_state_.robot_center_.header.frame_id = frame;
    set_complete_++;
}

void Robot::setCenter(const float x, const float y, const float z)
{
    robot_state_.robot_center_.transform.translation.x = x;
    robot_state_.robot_center_.transform.translation.y = y;
    robot_state_.robot_center_.transform.translation.z = z;
    set_complete_++;
}

void Robot::setVelocity(const float vx, const float vy, const float vz)
{
    robot_state_.v_x = vx;
    robot_state_.v_y = vy;
    robot_state_.v_z = vz;
    set_complete_++;
}

void Robot::setW(const float w)
{
    robot_state_.w = w;
    set_complete_++;
}

void Robot::setR(float r)
{
    robot_state_.r = r;
    set_complete_++;
}

void Robot::start() const
{
    assert(set_complete_ >= 8);
}

robot::RobotMarkers Robot::getState(const rclcpp::Time& time,bool iter)
{
    auto dt = (time - robot_state_.robot_center_.header.stamp).seconds();

    geometry_msgs::msg::Transform new_pose;
    new_pose.translation.x = robot_state_.robot_center_.transform.translation.x + dt * robot_state_.v_x;
    new_pose.translation.y = robot_state_.robot_center_.transform.translation.y + dt * robot_state_.v_y;
    new_pose.translation.z = robot_state_.robot_center_.transform.translation.z + dt * robot_state_.v_z;

    auto da = dt * robot_state_.w;
    Eigen::Matrix3d dR;
    dR << cos(da), sin(da), 0,
        -sin(da), cos(da), 0,
        0, 0, 1;
    Eigen::Matrix3d r = robot_state_.R * dR;

    if (iter)/*!更新参数*/
    {
        robot_state_.robot_center_.header.stamp = time;
        robot_state_.robot_center_.transform.translation = new_pose.translation;
        robot_state_.R = r;
    }

    Eigen::Matrix3d r90;
    r90 << 0, 1, 0,
        -1, 0, 0,
        0, 0, 1;

    RobotMarkers robot_markers;
    robot_markers.type = robot_type_;
    robot_markers.frame = iter?frame_:"self";
    robot_markers.center_transform = robot_state_.robot_center_;
    robot_markers.center_transform.transform = new_pose;
    Eigen::Vector3d armor_vec;
    armor_vec << robot_state_.r, 0, 0;
    bool is_once = false;

    for (auto i = 0; i < 4; i++)
    {
        r = r * r90;
        auto arm_position = r * armor_vec;
        auto dot = sight_dir_->dot(arm_position);

        Marker armor_marker;
        if (dot < -0.5)
            armor_marker = preset_.visible_armor;
        else
            armor_marker = preset_.invisible_armor;
        armor_marker.header.frame_id = iter?frame_:"self";
        armor_marker.header.stamp = time;

        armor_marker.id = i;
        armor_marker.pose.position.x = arm_position(0);
        armor_marker.pose.position.y = arm_position(1);
        armor_marker.pose.position.z = arm_position(2);
        //r = Eigen::Matrix3d::Identity();
        Eigen::Quaterniond q(r * armor_R_);
        armor_marker.pose.orientation.x = q.x();
        armor_marker.pose.orientation.y = q.y();
        armor_marker.pose.orientation.z = q.z();
        armor_marker.pose.orientation.w = q.w();
        robot_markers.markers.markers.push_back(armor_marker);
        robot::Armor armor;
        armor.armor = armor_marker;
        if (dot < -0.5)
            armor.visual = true;
        else
            armor.visual = false;
        robot_markers.armors.push_back(armor);

        if (!is_once)
        {
            Marker center_sphere = preset_.center;
            center_sphere.header.frame_id = iter?frame_:"self";
            center_sphere.header.stamp = time;
            robot_markers.markers.markers.push_back(center_sphere);
            is_once = true;
        }
    }
    robot_markers.label.vx = robot_state_.v_x;
    robot_markers.label.vy = robot_state_.v_y;
    robot_markers.label.vz = robot_state_.v_z;
    robot_markers.label.w = robot_state_.w;
    robot_markers.label.r1 = robot_state_.r;
    robot_markers.label.r2 = robot_state_.r;
    return robot_markers;
}

void Robot::initMarkers()
{
    preset_.visible_armor.header.frame_id = frame_;
    preset_.visible_armor.ns = "armor";
    preset_.visible_armor.action = Marker::ADD;
    preset_.visible_armor.type = Marker::CUBE;
    preset_.visible_armor.color.g = 1.0;
    preset_.visible_armor.color.b = 0.0;
    preset_.visible_armor.color.r = 0.0;
    preset_.visible_armor.color.a = 1.0;
    preset_.visible_armor.scale.x = 0.02;
    preset_.visible_armor.scale.y = 0.4;
    preset_.visible_armor.scale.z = 0.2;

    preset_.invisible_armor.header.frame_id = frame_;
    preset_.invisible_armor.ns = "armor";
    preset_.invisible_armor.action = Marker::ADD;
    preset_.invisible_armor.type = Marker::CUBE;
    preset_.invisible_armor.color.g = 0.0;
    preset_.invisible_armor.color.b = 0.0;
    preset_.invisible_armor.color.r = 1.0;
    preset_.invisible_armor.color.a = 1.0;
    preset_.invisible_armor.scale.x = 0.02;
    preset_.invisible_armor.scale.y = 0.4;
    preset_.invisible_armor.scale.z = 0.2;

    preset_.center.header.frame_id = frame_;
    preset_.center.ns = "center";
    preset_.center.action = Marker::ADD;
    preset_.center.type = Marker::SPHERE;
    preset_.center.scale.x = preset_.center.scale.y = preset_.center.scale.z = 0.2;
    preset_.center.color.a = 1.0;
    preset_.center.color.g = 1.0;
}