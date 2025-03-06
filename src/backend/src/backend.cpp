//
// Created by lbw on 25-3-2.
//

#include "backend.h"

BackEnd::BackEnd(): rclcpp::Node("backend")
{
    RCLCPP_INFO(this->get_logger(), "BackEndNode is Started!");
    esl_markers_pub_ = this->create_publisher<MarkerArray>("backend/esl_markers", 10);

    armors_sub_ = this->create_subscription<Armors>("environment/armors", 10,
                                                    std::bind(&BackEnd::armors_callback, this, std::placeholders::_1));
    change_state_sub_ = this->create_subscription<String>("environment/change_state", 10,
        std::bind(&BackEnd::change_state_callback, this, std::placeholders::_1));

    predict_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                             std::bind(&BackEnd::predict_timer_callback, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    self_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    init();
}

BackEnd::~BackEnd() = default;

void BackEnd::init()
{
    initMarkers();
    initMemory();
}

void BackEnd::initMarkers()
{
    auto& center = preset_.center_esl;
    center.header.frame_id = "self";
    center.header.stamp = this->get_clock()->now();
    center.action = Marker::ADD;
    center.type = Marker::SPHERE;
    center.scale.x = center.scale.y = center.scale.z = 0.2;
    center.color.a = 1.0;
    center.color.r = 1.0;
    center.color.g = 1.0;
    center.color.b = 0.0;

    auto& armor = preset_.armor;
    armor.header.frame_id = "self";
    armor.header.stamp = this->get_clock()->now();
    armor.action = Marker::ADD;
    armor.type = Marker::CUBE;
    armor.scale.x = 0.02;
    armor.scale.y = 0.4;
    armor.scale.z = 0.2;
    armor.color.a = 1.0;
    armor.color.r = 1.0;
    armor.color.g = 1.0;
    armor.color.b = 0.0;
}

void BackEnd::initMemory()
{
    nn::TrainParam param;
    param.batch_size = 64;
    param.epochs = 2;
    param.episodes = 3000;
    param.stepstamps = 200;
    param.lr = 0.003;
    param.time_step = 10;
    param.train_rate = 0.9;
    param.input_state = 8;
    param.output_state = 6;
    param.input_size = 80;
    param.output_size = 6;
    param.save_path = "/home/lbw/RM2025/Backend-simulator/src/backend/outputModels";

    // 声明网络
    memory_ = std::make_unique<Memory>(param);
    std::unique_ptr<MPTrainer> trainer = std::make_unique<MPTrainer>(param);
    std::unique_ptr<nn_MP> net = std::make_unique<nn_MP>(param);
    std::unique_ptr<torch::nn::MSELoss> mse_loss =
        std::make_unique<torch::nn::MSELoss>(
        torch::nn::MSELossOptions().reduction(torch::kMean));
    std::unique_ptr<torch::optim::Adam> optimizer =
        std::make_unique<torch::optim::Adam>(net->parameters(), param.lr);
    trainer->setNet(std::move(net));
    trainer->setLossFunction(std::move(mse_loss));
    trainer->setOptimizer(std::move(optimizer));
    trainer->setWriter("/home/lbw/RM2025/Backend-simulator/src/backend/outputModels");

    memory_->registerTrainer(std::move(trainer));
}

void BackEnd::armors_callback(const Armors::SharedPtr msg)
{
    armors_ = *msg;
    memory_->push(armors_);
}

void BackEnd::change_state_callback(const String::ConstSharedPtr msg)
{
    PUT_DEBUG("I heared: [" << msg->data << "]\n");
    memory_->forceUpload();
}

void BackEnd::predict_timer_callback()
{
    std::string to_frame = "self";
    std::string from_frame = armors_.header.frame_id;
    TransformStamp transform;
    try
    {
        transform = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
                     from_frame.c_str(), to_frame.c_str(), ex.what());
        return;
    }
    MarkerArray esl_markers;
    //set Center phere
    Marker esl_center = preset_.center_esl;
    esl_center.header.frame_id = "self";
    esl_center.ns = "esl_center";
    esl_center.header.stamp = this->get_clock()->now();
    esl_center.pose.position.x = transform.transform.translation.x;
    esl_center.pose.position.y = transform.transform.translation.y;
    esl_center.pose.position.z = transform.transform.translation.z;
    esl_markers.markers.push_back(esl_center);

    for (auto& armor : armors_.armors)
    {
        Marker realArmor = preset_.armor;
        realArmor.header.stamp = armor.header.stamp;
        realArmor.header.frame_id = "self";
        realArmor.ns = "esl_armor";
        realArmor.pose = armor.pose;
        realArmor.pose.position.x += transform.transform.translation.x;
        realArmor.pose.position.y += transform.transform.translation.y;
        realArmor.pose.position.z += transform.transform.translation.z;

        esl_markers.markers.push_back(realArmor);
    }
    // RCLCPP_INFO(this->get_logger(), "Num of visible Armor is %lu",esl_markers.markers.size());
    esl_markers_pub_->publish(esl_markers);
}



