//
// Created by lbw on 25-3-2.
//

#include "environment/Environment.h"

Environment::Environment(): rclcpp::Node("Environment")
{
    RCLCPP_INFO(this->get_logger(), "EnvironmentNode is Started!");
    self_markers_pub_ = this->create_publisher<MarkerArray>("environment/self", 10);
    robot_markers_pub_ = this->create_publisher<MarkerArray>("environment/robot", 10);
    next_robot_markers_pub_ = this->create_publisher<MarkerArray>("environment/next", 10);
    armors_pub_ = this->create_publisher<Armors>("environment/armors", 10);
    change_state_pub_ = this->create_publisher<std_msgs::msg::String>("environment/change_state", 10);

    marker_timer_ = this->create_wall_timer(std::chrono::milliseconds(5),
                                            std::bind(&Environment::publish_markers, this));
    trans_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                           std::bind(&Environment::change_state, this));
    odom_to_self_broadcast_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    self_to_robot_broadcast_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    init();
}

Environment::~Environment() = default;

void Environment::init()
{
    int width = this->declare_parameter("site/width", 1);
    int length = this->declare_parameter("site/length", 1);
    site_param_ = {width, length};
    sight_dir_ = std::make_shared<Robot::DirVector>();
    *sight_dir_ << 1, 0, 0;
    initMarkers();
    auto robot = initRobot("sentry", 3.5, 0, 0, 1, 0, 0, 0, 2);

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

std::unique_ptr<Robot> Environment::initRobot(const std::string& type,
                                              float x, float y, float z, float r,
                                              float vx, float vy, float vz, float w)
{
    auto robot = std::make_unique<Robot>("robot");
    robot->setRobotType(type);
    robot->setCenter(x, y, z);
    robot->setR(r);
    robot->setVelocity(vx, vy, vz);
    robot->setW(w);
    robot->setParent("self");
    robot->setSight(sight_dir_);
    robot->setTimestamp(this->get_clock()->now());
    robot->start();
    return robot;
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
        auto nextRobotState = robot->getState(this->get_clock()->now() + rclcpp::Duration(5, 0), false);
        if (!once)
        {
            self_to_robot_broadcast_->sendTransform(robotState.center_transform);
            once = true;
            for (auto& armor_marker : nextRobotState.markers.markers)
            {
                armor_marker.pose.position.x += nextRobotState.center_transform.transform.translation.x;
                armor_marker.pose.position.y += nextRobotState.center_transform.transform.translation.y;
                armor_marker.pose.position.z += nextRobotState.center_transform.transform.translation.z;
            }
            for (auto& armor : nextRobotState.armors)
            {
                armor.armor.pose.position.x += nextRobotState.center_transform.transform.translation.x;
                armor.armor.pose.position.y += nextRobotState.center_transform.transform.translation.y;
                armor.armor.pose.position.z += nextRobotState.center_transform.transform.translation.z;
            }
            nextRobotState.center.pose.position.x += nextRobotState.center_transform.transform.translation.x;
            nextRobotState.center.pose.position.y += nextRobotState.center_transform.transform.translation.y;
            nextRobotState.center.pose.position.z += nextRobotState.center_transform.transform.translation.z;
        }
        robot_markers_pub_->publish(robotState.markers);
        next_robot_markers_pub_->publish(nextRobotState.markers);


        Armors armors_to_pub;
        armors_to_pub.header.stamp = this->get_clock()->now();
        armors_to_pub.header.frame_id = robotState.frame;
        armors_to_pub.type = robotState.type;
        for (int i = 0; i < robotState.armors.size(); i++)
        {
            auto& armor = robotState.armors[i];
            if (!armor.visual)
                continue;
            Armor armor_to_pub;
            armor_to_pub.header.stamp = armors_to_pub.header.stamp;
            armor_to_pub.header.frame_id = robotState.frame;
            armor_to_pub.type = robotState.type;
            armor_to_pub.pose = armor.armor.pose;
            armor_to_pub.id = armor.armor.id;
            armor_to_pub.next_pose = nextRobotState.armors[i].armor.pose;
            armor_to_pub.label = robotState.label;

            armors_to_pub.armors.push_back(armor_to_pub);
        }

        armors_pub_->publish(armors_to_pub);
        // RCLCPP_INFO_STREAM(get_logger(), "Armors published");
    }
}

void Environment::change_state()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist(-1000, +1000);
    for (const auto& robot : robot_)
    {
        double x = dist(gen) / 100.0;
        double y = dist(gen) / 100.0;
        double z = dist(gen) / 100.0;
        double vx = dist(gen) / 100.0;
        double vy = dist(gen) / 100.0;
        double vz = dist(gen) / 100.0;
        double w = dist(gen) / 100.0;
        double r = abs(dist(gen) / 100.0) + 3;
        robot->setCenter(x, y, z);
        robot->setR(r);
        robot->setVelocity(vx, vy, vz);
        robot->setW(w);
        std::cout << "x： " << x << std::endl;
        std::cout << "y: " << y << std::endl;
        std::cout << "z: " << z << std::endl;
        std::cout << "vx: " << vx << std::endl;
        std::cout << "vy: " << vy << std::endl;
        std::cout << "vz: " << vz << std::endl;
        std::cout << "w: " << w << std::endl;
        std::cout << "r: " << r << std::endl;
        RCLCPP_INFO(this->get_logger(), "Robot State Change");
    }
    std_msgs::msg::String msg;
    msg.data = "I change State, Please Upload Data";
    change_state_pub_->publish(msg);
}
