//
// Created by lbw on 25-3-3.
//

#ifndef NN_MP_H
#define NN_MP_H

#include <torch/torch.h>

class nn_MP : public torch::nn::Module{
public:
    nn_MP();
    explicit nn_MP(int input_time_len);
    ~nn_MP() override;

    torch::Tensor forward(torch::Tensor input,torch::Tensor dt);
private:
    int input_time_len_;
    torch::nn::Linear linear1{nullptr};
    torch::nn::Linear linear2{nullptr};
    torch::nn::Linear linear3{nullptr};

};



#endif //NN_MP_H
