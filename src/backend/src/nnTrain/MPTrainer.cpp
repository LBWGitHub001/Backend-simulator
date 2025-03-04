//
// Created by lbw on 25-3-3.
//
#include <torch/data.h>
#include "nnTrain/MPTrainer.h"

MPTrainer::MPTrainer(const nn::TrainParam& train_param)
{
    config_ = train_param;
}

MPTrainer::~MPTrainer()
{
    stop_ = true;
    while (busy_);
}

void MPTrainer::train_loop()
{
    busy_ = true;
    std::cout << "The train thread is Started!" << std::endl;
    while (!stop_)
    {
        if (dataset_queue_.empty())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        DataPair&& dp = std::move(dataset_queue_.front());
        //TODO 完成训练流程



        dataset_queue_.pop();
    }
    busy_ = false;
}

void MPTrainer::setNet(std::unique_ptr<nn_MP> net)
{
    if (net_ == nullptr)
        net_ = std::move(net);
}

void MPTrainer::setLossFunction(std::unique_ptr<torch::nn::MSELoss> lossFunction)
{
    if (lossFunction == nullptr)
        loss_function_ = std::move(lossFunction);
}

void MPTrainer::setOptimizer(std::unique_ptr<torch::optim::Adam> optimizer)
{
    if (optimizer == nullptr)
        optimizer_ = std::move(optimizer);
}

void MPTrainer::startTrain()
{
}

void MPTrainer::train()
{
    for (int epoch = 0; epoch < config_.epochs; epoch++)
    {
    }
}

void MPTrainer::upload(std::vector<nn::DataSet>& data_sets)
{
    std::cout << "Get Data...";
    int train_size = data_sets.size() * config_.train_rate;
    auto begin = data_sets.begin();
    auto train_set = std::vector(begin, begin + train_size);
    auto train_ds = MP::DataSet(train_set);
    auto train_loader = torch::data::make_data_loader<torch::data::samplers::RandomSampler>(
        std::move(train_ds),
        torch::data::DataLoaderOptions().batch_size(64).drop_last(false));

    auto val_set = std::vector(begin + train_size, data_sets.end());
    auto val_ds = MP::DataSet(val_set);
    auto val_loader = torch::data::make_data_loader<torch::data::samplers::SequentialSampler>(std::move(val_ds),
        torch::data::DataLoaderOptions().batch_size(64).drop_last(false));
    DataPair dp;
    dp.train_loader = std::move(train_loader);
    dp.val_loader = std::move(val_loader);
    dataset_queue_.push(std::move(dp));
    std::cout << " Complete!" << std::endl;
    if (train_thread_ == nullptr)
        train_thread_ = std::make_unique<std::thread>(&MPTrainer::train_loop, this);
}
