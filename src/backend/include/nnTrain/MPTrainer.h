//
// Created by lbw on 25-3-3.
//

#ifndef MPTRAINER_H
#define MPTRAINER_H
#include <torch/torch.h>
#include <memory>
#include <queue>
#include <thread>
#include "nnTrain/nn_common.h"
#include "nnTrain/nn_MP.h"
#include "nnTrain/train_base.h"


class MPTrainer
{
public:
    struct DataPair
    {
        std::unique_ptr<torch::data::StatelessDataLoader<MP::DataSet,int>> train_loader{nullptr};
        std::unique_ptr<torch::data::StatelessDataLoader<MP::DataSet,int>> val_loader{nullptr};
    };
public:
    MPTrainer(const nn::TrainParam& train_param);
    ~MPTrainer();

    void setNet(std::unique_ptr<nn_MP> net);
    void setLossFunction(std::unique_ptr<torch::nn::MSELoss> lossFunction);
    void setOptimizer(std::unique_ptr<torch::optim::Adam> optimizer);

    void startTrain();
    void train();
    void upload(std::vector<nn::DataSet>& data_sets);

private:
    //训练配置
    nn::TrainParam config_;
    //训练组件
    std::unique_ptr<nn_MP> net_{nullptr};
    std::unique_ptr<torch::nn::MSELoss>loss_function_{nullptr};
    std::unique_ptr<torch::optim::Adam> optimizer_{nullptr};
    bool stop_;
    bool busy_;
    void train_loop();
    std::unique_ptr<std::thread> train_thread_{nullptr};

    //训练数据管理
    std::queue<DataPair> dataset_queue_;
};


#endif //MPTRAINER_H
