//
// Created by lbw on 25-3-3.
//

#ifndef NN_COMMON_H
#define NN_COMMON_H
#include <torch/torch.h>
#include <interfaces/msg/label.hpp>
#include "nn_common.h"

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
        double train_rate;/*!训练集在所有数据中的占比*/
        int input_state;/*!输入的状态量个数*/
        int output_state;/*!输出的状态量个数*/
        int input_size;/*!输入模型的大小*/
        int output_size;
    };

    struct Label
    {
        double w;
        double vx, vy, vz;
        double r1, r2;
    };


    struct DataSet
    {
        torch::Tensor data;
        torch::Tensor label;
    };


    struct lazyDataSet__struct
    {
        std::vector<torch::Tensor> data;
        std::vector<Label> labels;
        std::vector<double> predict_step;
    };

    using lazyDataSet__id = std::map<int, lazyDataSet__struct>;
    using lazyDataSet = std::map<std::string, lazyDataSet__id>;

    inline Label interface_to_nnType(interfaces::msg::Label label)
    {
        Label label_;
        label_.vx = label.vx;
        label_.vy = label.vy;
        label_.vz = label.vz;
        label_.w = label.w;
        label_.r1 = label.r1;
        label_.r2 = label.r2;
        return label_;
    }

    class TensorShape
    {
        public:
        TensorShape(torch::Tensor& tensor)
        {
            auto tshape = tensor.sizes();
            for (auto i: tshape)
            {
                shape.push_back(i);
            }
        }
        std::string getString()
        {
            std::string ss;
            ss+="[";
            for (auto& item : shape)
            {
                ss+=std::to_string(item);
                ss+=",";
            }
            ss += "]";
            return ss;
        }
        std::vector<long> shape;
    };
}
#endif //NN_COMMON_H
