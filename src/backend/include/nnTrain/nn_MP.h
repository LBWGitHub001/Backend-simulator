//
// Created by lbw on 25-3-3.
//

#ifndef NN_MP_H
#define NN_MP_H

#include <torch/torch.h>
#include "nnTrain/nn_common.h"


class nn_MP : public torch::nn::Module
{
public:
    nn_MP();
    explicit nn_MP(nn::TrainParam config);
    ~nn_MP() override;

    torch::Tensor forward(torch::Tensor input);

private:
    nn::TrainParam config_;
    int input_time_len_;
    torch::nn::Linear linear1{nullptr};
    torch::nn::Linear linear2{nullptr};
    torch::nn::Linear linear3{nullptr};
};

namespace MP
{
    class DataSet : public torch::data::datasets::Dataset<DataSet>
    {
    public:
        explicit DataSet(std::vector<nn::DataSet>& dataset): dataset_(dataset)
        {
        }

        ~DataSet() override = default;

        torch::data::Example<> get(size_t index) override
        {
            auto [data, label] = dataset_[index];
            return {data, label};
        }

        std::optional<size_t> size() const override
        {
            return dataset_.size();
        }

    private:
        std::vector<nn::DataSet> dataset_;
    };
}

#endif //NN_MP_H
