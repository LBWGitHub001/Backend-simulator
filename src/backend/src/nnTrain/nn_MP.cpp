//
// Created by lbw on 25-3-3.
//

#include "nnTrain/nn_MP.h"

nn_MP::nn_MP()
{
    assert("You Lose few parameters");
}

nn_MP::nn_MP(nn::TrainParam config)
{
    config_ = config;
    input_time_len_ = config_.time_step;
    linear1 = register_module("linear1", torch::nn::Linear(config_.input_size, 60));
    linear2 = register_module("linear2", torch::nn::Linear(60, 30));
    linear3 = register_module("linear3", torch::nn::Linear(30, config_.output_size));
}

nn_MP::~nn_MP() = default;

torch::Tensor nn_MP::forward(torch::Tensor input)
{
    auto x = input.reshape({-1, config_.input_size});
    x = torch::relu(linear1->forward(x));
    x = torch::relu(linear2->forward(x));
    x = torch::relu(linear3->forward(x));
    return x;
}


