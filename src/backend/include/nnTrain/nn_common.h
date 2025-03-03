//
// Created by lbw on 25-3-3.
//

#ifndef NN_COMMON_H
#define NN_COMMON_H
#include <torch/torch.h>

namespace nn{
    struct TrainParam{
    int epochs;
    int eposides;
    int stepstamps;
    int batch_size;
    double lr;
        int time_step;

    };

    struct DataSet
    {
        std::vector<torch::Tensor> data;
        std::vector<torch::Tensor> label;
    };

}
#endif //NN_COMMON_H
