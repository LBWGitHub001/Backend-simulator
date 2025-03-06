//
// Created by lbw on 25-3-3.
//
#include <torch/data.h>
#include <cuda_runtime.h>
#include "nnTrain/MPTrainer.h"

MPTrainer::MPTrainer(const nn::TrainParam& train_param)
{
    config_ = train_param;
    checkGPU();
}

MPTrainer::~MPTrainer()
{
    stop_ = true;
    while (busy_) { std::this_thread::sleep_for(std::chrono::seconds(1)); }
}

void MPTrainer::checkGPU()
{
    if (torch::cuda::is_available())
    {
        std::cout << "-------------------------------------------------" << std::endl;
        std::cout << "CUDA is available! GPU acceleration enabled." << std::endl;
        // 获取设备数量[5]
        int device_count = torch::cuda::device_count();
        std::cout << "Detected " << device_count << " CUDA-capable device(s)" << std::endl;

        // 遍历所有设备[5]
        for (int i = 0; i < device_count; ++i)
        {
            size_t free_mem, total_mem;
            cudaMemGetInfo(&free_mem, &total_mem);
            std::cout << "  VRAM: " << (total_mem >> 20) << "MB (Total) / "
                << (free_mem >> 20) << "MB (Free)" << std::endl;
        }
        std::cout << "-------------------------------------------------" << std::endl;
        device_ = torch::kCUDA;
    }
    else
    {
        std::cerr << "CUDA is not available. Using CPU." << std::endl;
        device_ = torch::kCPU;
    }
}

void MPTrainer::train_loop()
{
    busy_ = true;
    stop_ = false;
    if (net_ == nullptr || loss_function_ == nullptr || optimizer_ == nullptr)
    {
        LOG_FATAL("Not all settings completed!");
        assert(false);
    }
    net_->train();
    LOG_INFO("The train thread is Started!");
    while (!stop_)
    {
        train();
    }
    busy_ = false;
}

void MPTrainer::setNet(std::unique_ptr<nn_MP> net)
{
    if (net_ == nullptr)
        net_ = std::move(net);
    net_->to(device_);
}

void MPTrainer::setLossFunction(std::unique_ptr<torch::nn::MSELoss> lossFunction)
{
    if (loss_function_ == nullptr)
        loss_function_ = std::move(lossFunction);
}

void MPTrainer::setOptimizer(std::unique_ptr<torch::optim::Adam> optimizer)
{
    if (optimizer_ == nullptr)
        optimizer_ = std::move(optimizer);
}

void MPTrainer::setWriter(const std::string& save_path_)
{
    writer_ = std::make_unique<Writer>(save_path_);
}


void MPTrainer::train()
{
    if (dataset_queue_.empty())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return;
    }
    DataPair&& dp = std::move(dataset_queue_.front());
    for (int epoch = 0; epoch < config_.epochs; epoch++)
    {
        double total_loss = 0.0;
        int batch_count = 0;
        *writer_ << "Train epoch: \n";
        for (auto& batch : *dp.train_loader)
        {
            double loss = aEpoch(batch);
            total_loss += loss;
            batch_count++;
            // std::cout << "Loss: " << loss << std::endl;
        }
        double train_loss_avg = total_loss / batch_count;

        total_loss = 0.0;
        batch_count = 0;
        *writer_ << "Vaild epoch: \n";
        for (auto& batch : *dp.val_loader)
        {
            total_loss += aEpoch(batch, false);
            batch_count++;
        }
        double val_loss_avg = total_loss / batch_count;
        PUT_WARN("Loss[train/val]: " << train_loss_avg << "/" << val_loss_avg << std::endl);
        *writer_ << "Loss[train/val]: " << train_loss_avg << "/" << val_loss_avg << "\n";
    }
    net_ = writer_->save(std::move(net_));
    dataset_queue_.pop();
}

double MPTrainer::aEpoch(std::vector<torch::data::Example<>>& batch, bool train)
{
    optimizer_->zero_grad();
    std::vector<torch::Tensor> x_vec;
    std::vector<torch::Tensor> y_vec;
    for (auto& singal : batch)
    {
        x_vec.push_back(singal.data);
        y_vec.push_back(singal.target);
    }
    auto xbatch = torch::cat(x_vec, -1);
    auto ybatch = torch::cat(y_vec, -1);
    auto x = xbatch.reshape({-1, config_.input_size}).to(device_);
    auto y = ybatch.reshape({-1, config_.output_size}).to(device_);
    auto y_hat = net_->forward(x);
    auto loss = (*loss_function_)->forward(y, y_hat);
    *writer_ << loss.item<double>() << "\n";
    if (train)
    {
        loss.backward();
        optimizer_->step();
    }
    return loss.item<double>();
}

void MPTrainer::upload(std::vector<nn::DataSet>& data_sets)
{
    PUT_INFO("Parsing Data...\t");
    auto start_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    int train_size = data_sets.size() * config_.train_rate;
    auto begin = data_sets.begin();
    auto train_set = std::vector(begin, begin + train_size);
    auto train_ds = MP::DataSet(train_set);
    auto train_loader = torch::data::make_data_loader<torch::data::samplers::RandomSampler>(
        std::move(train_ds),
        torch::data::DataLoaderOptions().batch_size(config_.batch_size).drop_last(false));

    auto val_set = std::vector(begin + train_size, data_sets.end());
    auto val_ds = MP::DataSet(val_set);
    auto val_loader = torch::data::make_data_loader<torch::data::samplers::SequentialSampler>(std::move(val_ds),
        torch::data::DataLoaderOptions().batch_size(config_.batch_size).drop_last(false));
    DataPair dp;
    dp.train_loader = std::move(train_loader);
    dp.val_loader = std::move(val_loader);
    dataset_queue_.push(std::move(dp));

    auto end_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    auto duration = end_time - start_time;
    PUT_INFO("Complete! It cost " << duration << " ms" << std::endl);

    if (train_thread_ == nullptr)
        train_thread_ = std::make_unique<std::thread>(&MPTrainer::train_loop, this);
}
