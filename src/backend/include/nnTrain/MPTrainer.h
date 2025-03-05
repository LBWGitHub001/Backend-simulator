//
// Created by lbw on 25-3-3.
//

#ifndef MPTRAINER_H
#define MPTRAINER_H
#include <torch/torch.h>
#include <memory>
#include <queue>
#include <thread>
#include "logger/logger.h"
#include "nnTrain/nn_common.h"
#include "nnTrain/nn_MP.h"


class MPTrainer
{
public:
    struct DataPair
    {
        std::unique_ptr<torch::data::StatelessDataLoader<MP::DataSet, torch::data::samplers::RandomSampler>>
        train_loader{nullptr};
        std::unique_ptr<torch::data::StatelessDataLoader<MP::DataSet, torch::data::samplers::SequentialSampler>>
        val_loader{nullptr};
        ~DataPair()
        {
            if (train_loader != nullptr)
            {
                train_loader.reset();
                val_loader.reset();
            }
        }
        DataPair() = default;

        DataPair(DataPair&& other) noexcept
        {
            train_loader = std::move(other.train_loader);
            val_loader = std::move(other.val_loader);
        }
        DataPair& operator=(DataPair&& other) noexcept
        {
            train_loader = std::move(other.train_loader);
            val_loader = std::move(other.val_loader);
            return *this;
        }
    };

public:
    explicit MPTrainer(const nn::TrainParam& train_param);
    ~MPTrainer();
    void checkGPU();

    void setNet(std::unique_ptr<nn_MP> net);
    void setLossFunction(std::unique_ptr<torch::nn::MSELoss> lossFunction);
    void setOptimizer(std::unique_ptr<torch::optim::Adam> optimizer);

    void upload(std::vector<nn::DataSet>& data_sets);

private:
    //训练配置
    nn::TrainParam config_{};
    c10::DeviceType device_;
    //训练组件
    std::unique_ptr<nn_MP> net_{nullptr};
    std::unique_ptr<torch::nn::MSELoss> loss_function_{nullptr};
    std::unique_ptr<torch::optim::Adam> optimizer_{nullptr};
    bool stop_{};
    bool busy_{};
    void train_loop();
    void train();
    double aEpoch(std::vector<torch::data::Example<>>& batch,bool train = true);
    std::unique_ptr<std::thread> train_thread_{nullptr};

    //训练数据管理
    std::queue<DataPair> dataset_queue_;

};


#endif //MPTRAINER_H
