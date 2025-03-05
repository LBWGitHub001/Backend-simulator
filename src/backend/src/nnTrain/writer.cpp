//
// Created by lbw on 25-3-4.
//

#include "nnTrain/writer.h"
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
    //torch::save(*net, model_path);
    torch::serialize::OutputArchive archive;
    for (const auto& pair : net->named_parameters())
    {
        const std::string& key = pair.key();
        const torch::Tensor& param = pair.value();
        archive.write(key, param);
    }
    archive.save_to(model_path);
    PUT_INFO("Saved model to " << model_path << std::endl);

    return net;
}

