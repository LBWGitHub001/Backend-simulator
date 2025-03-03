//
// Created by lbw on 25-3-3.
//

#ifndef NN_COMMON_H
#define NN_COMMON_H
#include <torch/torch.h>

namespace nn
{
    struct TrainParam
    {
        int epochs; /*!同一批数据更新网络的次数*/
        int episodes; /*!Agent与环境交互多少次*/
        int stepstamps; /*!收集多少数据后更新参数*/
        int batch_size; /*!训练批次*/
        double lr; /*!学习率*/
        int time_step; /*!用多少数据来预测接下来的动作*/
    };

    struct DataSet
    {
        torch::Tensor data;
        torch::Tensor label;
    };

    struct lazyDataSet
    {
        std::vector<torch::Tensor> data;
        std::vector<torch::Tensor> label;
        std::vector<double> predict_step;
    };
}
#endif //NN_COMMON_H
