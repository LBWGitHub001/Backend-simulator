//
// Created by lbw on 25-3-3.
//

#include "memory.h"

Memory::Memory(const nn::TrainParam& train_param): start_time_(0)
{
    config_ = train_param;
    state_ = SelectState::First;
}

Memory::~Memory()
{
}

void Memory::registerTrainer(std::unique_ptr<TrainBase> trainer)
{
    trainer_ = std::move(trainer);
}

void Memory::push(interfaces::msg::Armors armors)
{
    auto timestamp = armors.header.stamp.nanosec / 1000.0;
    if (state_ == SelectState::First)
    {
        start_time_ = timestamp;
        input_buffer_.data.clear();
        input_buffer_.predict_step.clear();
        state_ = SelectState::Selecting;
    }
    timestamp -= start_time_;

    for (auto& armor : armors.armors)
    {
        std::vector<double> armor_state;
        armor_state.push_back(timestamp);
        armor_state.push_back(armor.pose.position.x);
        armor_state.push_back(armor.pose.position.y);
        armor_state.push_back(armor.pose.position.z);
        armor_state.push_back(armor.pose.orientation.x);
        armor_state.push_back(armor.pose.orientation.y);
        armor_state.push_back(armor.pose.orientation.z);
        armor_state.push_back(armor.pose.orientation.w);
        input_buffer_.data.push_back(torch::tensor(armor_state));
        input_buffer_.predict_step.push_back(armor.predict_step);
    }

    if (input_buffer_.data.size() == config_.stepstamps)
    {
        std::thread start_update(&Memory::update, this);
        state_ = SelectState::Updating;
        start_update.detach();
    }
}

void Memory::update()
{
    nn::DataSet data_set;
    auto start = input_buffer_.data.begin();
    auto end = start + config_.time_step;
    int predict_step_index = config_.time_step;

    std::vector<torch::Tensor> dataset;
    while (end != input_buffer_.data.end())
    {
        std::vector<torch::Tensor> input_vec(start, end);
        double predict_step = input_buffer_.predict_step.at(predict_step_index);
        input_vec.push_back(torch::tensor({predict_step}));
        auto input_tr = torch::cat(input_vec, 1);
        dataset.push_back(input_tr);
        ++start, ++end,++predict_step_index;
    }

    state_ = SelectState::First;
}
