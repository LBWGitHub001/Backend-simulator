//
// Created by lbw on 25-3-2.
//

#include "environment/Environment.h"

Environment::Environment(): rclcpp::Node("Environment")
{
    self_markers_pub_ = this->create_publisher<MarkerArray>("environment/self", 10);
    robot_markers_pub_ = this->create_publisher<MarkerArray>("environment/robot", 10);

    marker_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                            std::bind(&Environment::publish_markers, this));
    odom_to_self_broadcast_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    self_to_robot_broadcast_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    init();
}

Environment::~Environment()
{
}

void Environment::init()
{
    int width = this->declare_parameter("site/width", 1);
    int length = this->declare_parameter("site/length", 1);
    site_param_ = {.width = width, .length = length};
    sight_dir_ = std::make_shared<Robot::DirVector>();
    *sight_dir_ << 1, 0, 0;
    initMarkers();
    auto robot = initRobot(3.5, 0, 0, 1, 0, 0, 0, 2);

    robot_.push_back(std::move(robot));
}

void Environment::initMarkers()
{
    odom_to_self.header.stamp = this->get_clock()->now();
    odom_to_self.header.frame_id = "root";
    odom_to_self.child_frame_id = "self";
    odom_to_self.transform.translation.x = 0;
    odom_to_self.transform.translation.y = 0;
    odom_to_self.transform.translation.z = 0;
    odom_to_self.transform.rotation.x = 0;
    odom_to_self.transform.rotation.y = 0;
    odom_to_self.transform.rotation.z = 0;
    odom_to_self.transform.rotation.w = 1;

    auto& ground = visual_elements_.ground;
    ground.header.stamp = this->get_clock()->now();
    ground.header.frame_id = "root";
    ground.id = 0;
    ground.type = Marker::CUBE;
    ground.action = Marker::ADD;
    ground.scale.x = site_param_.length;
    ground.scale.y = site_param_.width;
    ground.scale.z = 1;
    ground.color.a = 1.0;
    ground.color.r = 0.0;
    ground.color.g = 0.0;
    ground.color.b = 1.0;
    ground.pose.position.x = 0.0;
    ground.pose.position.y = 0.0;
    ground.pose.position.z = -1.0;
    ground.pose.orientation.x = 0.0;
    ground.pose.orientation.y = 0.0;
    ground.pose.orientation.z = 0.0;
    ground.pose.orientation.w = 1.0;

    auto& sight = visual_elements_.sight;
    sight.header.stamp = this->get_clock()->now();
    sight.header.frame_id = "self";
    sight.id = 0;
    sight.type = Marker::ARROW;
    sight.action = Marker::ADD;
    sight.scale.x = 2;
    sight.scale.y = 0.04;
    sight.scale.z = 0.04;
    sight.color.a = 1.0;
    sight.color.r = 0.0;
    sight.color.g = 0.0;
    sight.color.b = 1.0;
    sight.pose.position.x = 0.0;
    sight.pose.position.y = 0.0;
    sight.pose.position.z = 0.0;
    sight.pose.orientation.x = 0.0;
    sight.pose.orientation.y = 0.0;
    sight.pose.orientation.z = 0.0;
    sight.pose.orientation.w = 1.0;
}

std::unique_ptr<Robot> Environment::initRobot(float x, float y, float z, float r,
                                              float vx, float vy, float vz, float w)
{
    auto robot = std::make_unique<Robot>("robot");
    robot->setCenter(x, y, z);
    robot->setR(r);
    robot->setVelocity(vx, vy, vz);
    robot->setW(w);
    robot->setParent("self");
    robot->setSight(sight_dir_);
    robot->setTimestamp(this->get_clock()->now());
    robot->start();
    return std::move(robot);
}

void Environment::publish_markers()
{
    //publish "self"
    MarkerArray markers;
    visual_elements_.ground.header.stamp = this->get_clock()->now();
    visual_elements_.sight.header.stamp = this->get_clock()->now();
    markers.markers.push_back(visual_elements_.ground);
    markers.markers.push_back(visual_elements_.sight);
    self_markers_pub_->publish(markers);

    odom_to_self.header.stamp = this->get_clock()->now();
    odom_to_self_broadcast_->sendTransform(odom_to_self);

    //publish "robot"
    bool once = false;
    MarkerArray markers_to_pub;
    for (auto& robot : robot_)
    {
        auto robotState = robot->getState(this->get_clock()->now());
        if (!once)
        {
            self_to_robot_broadcast_->sendTransform(robotState.center_transform);
            once = true;
        }
        // for (auto& marker : robotState.markers)
        // markers_to_pub.markers.push_back();
        robot_markers_pub_->publish(robotState.markers);
    }
    // robot_markers_pub_->publish(markers_to_pub);
}
