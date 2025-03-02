//
// Created by lbw on 25-1-16.
//

#include "Robot.h"

Robot::Robot(): Node("robot"), v_yaw_(1), yaw_(0), r_(0.4)
{
    height_ = declare_parameter("height", 0.2f);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(1),
        [this] { return timer_callback(); });

    robot_pub_ = create_publisher<RobotInfo>(
        "front_client/robot_status",
        10);

    armor_pub_ = create_publisher<Armors>(
        "front_client/visual_armor",
        10);

    markers_pub_ = create_publisher<MarkerArray>(
        "front_client/markers",
        10);

    center_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    camera_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timestamp_ = this->get_clock()->now();
    initRobot();
    initMarkers();

    camera_to_robot_.header.frame_id = "camera_to_robot";

    update_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&Robot::update_timer_callback, this));
}

void Robot::timer_callback()
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "camera";
    t.child_frame_id = "center";
    t.transform.translation.x = center_.x;
    t.transform.translation.y = center_.y;
    t.transform.translation.z = center_.z;
    center_broadcaster_->sendTransform(t);
    publishMarkers();
    publishRobotInfo();
}

void Robot::initRobot()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution dis(0.0, 3.0); // 定义一个范围在0.0到1.0的均匀分布

    //定义中心点的坐标系
    do
    {
         center_.x = dis(gen);
         center_.y = dis(gen);
    }while (center_.x*center_.x + center_.y*center_.y < 4.0 || center_.y<0.5);

    center_.z = height_;
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "camera";
    t.child_frame_id = "center";
    t.transform.translation.x = center_.x;
    t.transform.translation.y = center_.y;
    t.transform.translation.z = center_.z;
    center_broadcaster_->sendTransform(t);

    //定义相机坐标
    geometry_msgs::msg::TransformStamped tt;
    tt.header.stamp = this->get_clock()->now();
    tt.child_frame_id = "camera";
    tt.transform.translation.x = tt.transform.translation.y = tt.transform.translation.z = 0.0;
    tt.transform.rotation.x = 0.0;
    tt.transform.rotation.y = 0.0;
    tt.transform.rotation.z = 0.0;
    tt.transform.rotation.w = 1.0;
    camera_broadcaster_->sendTransform(tt);
}

void Robot::initMarkers()
{
    //中心圆球
    center_marker_.header.frame_id = "center";
    center_marker_.ns = "center";
    center_marker_.id = 0;
    center_marker_.header.stamp = this->get_clock()->now();
    center_marker_.type = Marker::SPHERE;
    center_marker_.scale.x = center_marker_.scale.y = center_marker_.scale.z = 0.2;
    center_marker_.color.a = 1.0;
    center_marker_.color.g = 1.0;

    //装甲板
    armor_marker_.header.frame_id = "center";
    armor_marker_.ns = "armor";
    armor_marker_.header.stamp = this->get_clock()->now();
    armor_marker_.type = Marker::CUBE;
    armor_marker_.color.g = 1.0;
    armor_marker_.color.b = 0.0;
    armor_marker_.color.r = 1.0;
    armor_marker_.color.a = 1.0;
    armor_marker_.scale.x = 0.02;
    armor_marker_.scale.y = 0.4;
    armor_marker_.scale.z = 0.2;

    //箭头
    robot_direction_marker_.header.frame_id = "camera";
    robot_direction_marker_.ns = "robot_direction";
    robot_direction_marker_.header.stamp = this->get_clock()->now();
    robot_direction_marker_.type = Marker::ARROW;
    robot_direction_marker_.color.g = 0.0;
    robot_direction_marker_.color.b = 0.0;
    robot_direction_marker_.color.r = 1.0;
    robot_direction_marker_.color.a = 1.0;
    robot_direction_marker_.scale.x = 0.7;
    robot_direction_marker_.scale.y = 0.04;
    robot_direction_marker_.scale.z = 0.04;
    robot_direction_marker_.pose.position.x = 0;
    robot_direction_marker_.pose.position.y = 0;
    robot_direction_marker_.pose.position.z = 0;
}

void Robot::publishMarkers()
{
    MarkerArray marker_array;

    auto T = this->get_clock()->now() - timestamp_;
    timestamp_ = this->get_clock()->now();

    center_marker_.header.stamp = timestamp_;
    center_marker_.action = Marker::ADD;
    marker_array.markers.push_back(center_marker_);

    camera_to_robot_direction();
    robot_direction_marker_.header.stamp = timestamp_;
    robot_direction_marker_.action = Marker::ADD;
    auto q = vector_to_quaternion(camera_to_robot_.vector);
    robot_direction_marker_.pose.orientation.w = q.w();
    robot_direction_marker_.pose.orientation.x = q.x();
    robot_direction_marker_.pose.orientation.y = q.y();
    robot_direction_marker_.pose.orientation.z = q.z();
    marker_array.markers.push_back(robot_direction_marker_);

    Armors armors;
    for (int i = 0; i < 4; i++)
    {
//        if (yaw_>M_PI/2)
//            yaw_-=M_PI*2;
        armor_marker_.action = Marker::ADD;
        yaw_ += v_yaw_ * T.seconds();
        double tmp_yaw = yaw_ + i * M_PI / 2;
        armor_marker_.pose.position.x = r_ * cos(tmp_yaw);
        armor_marker_.pose.position.y = r_ * sin(tmp_yaw);

        tf2::Quaternion q;
        q.setRPY(0, -0.2, tmp_yaw);
        armor_marker_.pose.orientation.x = q.getX();
        armor_marker_.pose.orientation.y = q.getY();
        armor_marker_.pose.orientation.z = q.getZ();
        armor_marker_.pose.orientation.w = q.getW();
        armor_marker_.id = i;


        //可视化判断
        Eigen::Vector3d armor_vector{cos(tmp_yaw),sin(tmp_yaw),0};
        Eigen::Vector3d robot_vector{camera_to_robot_.vector.x,camera_to_robot_.vector.y,0};
        auto cosAngle = armor_vector.dot(robot_vector);
        if (cosAngle>0.1)
        {
            Armor armor;
            armor.id = i;
            armor.pose.orientation.x = q.getX();
            armor.pose.orientation.y = q.getY();
            armor.pose.orientation.z = q.getZ();
            armor.pose.orientation.w = q.getW();
            armor.pose.position.x = r_ * cos(tmp_yaw);
            armor.pose.position.y = r_ * sin(tmp_yaw);
            RCLCPP_INFO_STREAM(this->get_logger(),armor.armor.x  << "," << tmp_yaw);
            armor.armor.z = height_ / 2;
            armor.number = "3";

            armors.header.frame_id = "visual_armor";
            armors.header.stamp = this->get_clock()->now();
            armors.armors.push_back(armor);
            RCLCPP_INFO_STREAM(this->get_logger(), "A visual armor");
            armor_marker_.color.g = 1.0;
            armor_marker_.color.b = 0.0;
            armor_marker_.color.r = 0.0;
        }
        else
        {
            armor_marker_.color.g = 1.0;
            armor_marker_.color.b = 0.0;
            armor_marker_.color.r = 1.0;
        }
        marker_array.markers.push_back(armor_marker_);
    }
    markers_pub_->publish(marker_array);
    armor_pub_->publish(armors);

}

void Robot::publishRobotInfo()
{
    RobotInfo msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "center";
    msg.center = center_;
    msg.h = height_;
    for (int i = 0; i < 4; i++)
    {
        double tmp_yaw = yaw_ + i * M_PI / 2;

        tf2::Quaternion q;
        q.setRPY(0, -0.2, tmp_yaw);

        Armor armor;
        armor.pose.orientation.x = q.getX();
        armor.pose.orientation.y = q.getY();
        armor.pose.orientation.z = q.getZ();
        armor.pose.orientation.w = q.getW();
        armor.number = "3";

        msg.armors.push_back(armor);
    }

    robot_pub_->publish(msg);
}

void Robot::camera_to_robot_direction()
{
    std::string fromFrame = tf_camera;
    std::string toFrame = tf_robot_center;

    TransfromStamped t;
    try
    {
        t = tf_buffer_->lookupTransform(toFrame,fromFrame,tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
            fromFrame.c_str(), toFrame.c_str(), ex.what());
        return;
    }
    float delta_x = t.transform.translation.x;
    float delta_y = t.transform.translation.y;
    float delta_z = t.transform.translation.z;

    float delta_x2 = delta_x*delta_x;
    float delta_y2 = delta_y*delta_y;
    float delta_z2 = delta_z*delta_z;

    float mould = pow(delta_x2+delta_y2+delta_z2,1./3.);
    delta_x = delta_x / mould;
    delta_y = delta_y / mould;
    delta_z = delta_z / mould;
    camera_to_robot_.mould = 1;
    camera_to_robot_.vector.x = delta_x;
    camera_to_robot_.vector.y = delta_y;
    camera_to_robot_.vector.z = delta_z;

    camera_to_robot_.header.stamp = this->get_clock()->now();

}

tf2::Quaternion Robot::vector_to_quaternion(const geometry_msgs::msg::Vector3& v)
{
    tf2::Vector3 directionVector(v.x,v.y,v.z);
    directionVector.normalize();
    float yaw = atan2(directionVector.y(),directionVector.x());
    float pitch = atan2(directionVector.z(),directionVector.y());
    tf2::Quaternion quaternion;
    quaternion.setRPY(0,pitch,yaw);
    return quaternion;
}

void Robot::update_timer_callback()
{
    initRobot();
    initMarkers();
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("updateWriter");
    rclcpp::Client<UpdateWriter>::SharedPtr client =
      node->create_client<UpdateWriter>("writer/update_writer");

    auto request = std::make_shared<UpdateWriter::Request>();
    request->update_once = true;

    client->async_send_request(request);
}
