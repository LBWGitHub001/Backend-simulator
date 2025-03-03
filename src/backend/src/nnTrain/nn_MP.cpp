//
// Created by lbw on 25-3-3.
//

#include "nnTrain/nn_MP.h"

nn_MP::nn_MP()
{
    nn_MP(10);
}

nn_MP::nn_MP(int input_time_len)
{
    input_time_len_ = input_time_len;
    linear1 = register_module("linear1", torch::nn::Linear(input_time_len * 3 * 4, 60));
    linear2 = register_module("linear2", torch::nn::Linear(60, 30));
    linear3 = register_module("linear3", torch::nn::Linear(30, 12));
}

nn_MP::~nn_MP() = default;

torch::Tensor nn_MP::forward(torch::Tensor input, torch::Tensor dt)
{
    auto x = input.reshape({input.size(0), input_time_len_ * 12});
    x = torch::cat({x, dt}, 1);
    x = torch::relu(linear1->forward(x));
    x = torch::relu(linear2->forward(x));
    x = torch::relu(linear3->forward(x));
    return x;
}