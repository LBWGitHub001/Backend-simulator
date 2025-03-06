//
// Created by lbw on 25-3-4.
//

#include "nnTrain/writer.h"
#include <torch/script.h>
#include <torch/csrc/jit/api/module.h>
namespace fs = std::filesystem;

Writer::Writer(std::string path): dir_path_(path)
{
    init();
}

Writer::~Writer()
{
}

void Writer::init()
{
    if (!fs::exists(dir_path_))
    {
        LOG_WARN("Directory does not exist,create one");
        LOG_WARN(dir_path_.c_str());
        assert(fs::create_directory(dir_path_));
    }

    int max_index = 0;
    for (const auto& entry : fs::directory_iterator(dir_path_))
    {
        if (entry.is_directory())
        {
            int index = std::stoi(entry.path().filename().string());
            if (index > max_index)
                max_index = index;
        }
    }
    private_path_ = dir_path_ + "/" + std::to_string(max_index + 1);
    fs::create_directory(private_path_);
}


std::unique_ptr<nn_MP> Writer::save(std::unique_ptr<nn_MP> net)
{
    std::string model_path = private_path_ + "/" + std::to_string(save_count_++);
    std::string pt_model = model_path + ".pt";
    std::string onnx_model = model_path + ".onnx";
    std::string log_path = model_path + ".log";
    torch::serialize::OutputArchive archive;
    for (const auto& pair : net->named_parameters())
    {
        const std::string& key = pair.key();
        const torch::Tensor& param = pair.value();
        archive.write(key, param);
    }
    archive.save_to(pt_model);

    //保存为onnx
    // auto traced_model = torch::jit::trace(net);
    PUT_INFO("Saved model to " << pt_model << std::endl);
    std::fstream log_writer(log_path.c_str(),std::ios::out);
    for (auto& str : outputs_)
    {
        log_writer << str;
    }
    log_writer.close();
    outputs_.clear();
    return std::move(net);
}

Writer& Writer::operator<<(std::string& str)
{
    outputs_.push_back(str);
    return *this;
}

Writer& Writer::operator<<(const int& str)
{
    outputs_.push_back(std::to_string(str));
    return *this;
}

Writer& Writer::operator<<(const float& str)
{
    outputs_.push_back(std::to_string(str));
    return *this;
}

Writer& Writer::operator<<(const double& str)
{
    outputs_.push_back(std::to_string(str));
    return *this;
}

Writer& Writer::operator<<(const char* str)
{
    outputs_.push_back(std::string(str));
    return *this;
}

