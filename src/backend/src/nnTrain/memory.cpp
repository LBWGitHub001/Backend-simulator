//
// Created by lbw on 25-3-3.
//

#include "nnTrain/memory.h"

Memory::Memory(const nn::TrainParam& train_param): start_time_(0)
{
    config_ = train_param;
    state_ = First;
}

Memory::~Memory()
{
}

void Memory::initBuff()
{
    /*! type:
     * "1":1号装甲板
     * "2":2号装甲板
     * "3":3号装甲板
     * "4":4号装甲板
     * "5":5号装甲板
     * "sentry":哨兵装甲板
     * "outpost":前哨站装甲板
     * "base":基地装甲板
     */
    input_buffer_["1"] = nn::lazyDataSet__id();
    input_buffer_["2"] = nn::lazyDataSet__id();
    input_buffer_["3"] = nn::lazyDataSet__id();
    input_buffer_["4"] = nn::lazyDataSet__id();
    input_buffer_["5"] = nn::lazyDataSet__id();
    input_buffer_["sentry"] = nn::lazyDataSet__id();
    input_buffer_["output"] = nn::lazyDataSet__id();
    input_buffer_["base"] = nn::lazyDataSet__id();
}

void Memory::registerTrainer(std::unique_ptr<TrainBase> trainer)
{
    trainer_ = std::move(trainer);
}

/*!
 * @brief 传入一组装甲板和一个标签，将数据加入数据缓冲区，等待缓冲区满或者长时间不接受数据后，这里数据并传输给网络进行训练
 * @param armors 当前识别到的装甲板，是同一时间识别到的
 * @param label 当前机器人的运动状态，详细信息略
 */
void Memory::push(interfaces::msg::Armors armors)
{
    double timestamp = armors.header.stamp.nanosec / 1000.0;
    if (armors.type != last_type_)
    {
        std::cout << "Change to Lost" << std::endl;
        state_ = Lost;
        std::thread Upload(&Memory::upload, this, last_type_);
        Upload.detach();
        std::cout << "Change to First" << std::endl;
        state_ = First;
    }
    while (state_ == Uploading)
    {

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (state_ == First)
    {
        std::cout << "Change to Selecting" << std::endl;
        start_time_ = timestamp;
        last_type_ = armors.type;
        type_count_[last_type_] = 0;
        state_ = Selecting;
    }
    timestamp -= start_time_;
    auto& buffer = input_buffer_[last_type_];
    for (auto& armor : armors.armors)
    {
        std::vector<double> dataFrame;
        dataFrame.push_back(timestamp);
        dataFrame.push_back(armor.pose.position.x);
        dataFrame.push_back(armor.pose.position.y);
        dataFrame.push_back(armor.pose.position.z);
        dataFrame.push_back(armor.pose.orientation.x);
        dataFrame.push_back(armor.pose.orientation.y);
        dataFrame.push_back(armor.pose.orientation.z);
        dataFrame.push_back(armor.pose.orientation.w);
        buffer[armor.id].predict_step.push_back(armor.predict_step);
        buffer[armor.id].data.push_back(torch::tensor(dataFrame));
        buffer[armor.id].labels.push_back(nn::interface_to_nnType(armor.label) );
        type_count_[last_type_]++;
    }

    if (type_count_[last_type_] > config_.stepstamps)
    {
        std::cout << "Change to Uploading" << std::endl;
        state_ = Uploading;
        std::thread Upload(&Memory::upload, this, last_type_);
        Upload.detach();
    }
}

void Memory::upload(std::string type)
{
    std::cout << "Uploading " << type << " ..." << std::endl;
    auto start_time = std::chrono::system_clock::now();
    std::vector<nn::DataSet> datasets;
    auto buffer = input_buffer_[type];
    for (auto buff_pre_id : buffer)
    {
        auto start = buff_pre_id.second.data.begin();
        auto end = start + config_.time_step;
        int predict_step_index = config_.time_step;
        int label_index = predict_step_index;

        nn::DataSet ds;
        while (end != buff_pre_id.second.data.end())
        {
            std::vector<torch::Tensor> input_vec(start, end);
            double predict_step = buff_pre_id.second.predict_step.at(predict_step_index);
            input_vec.push_back(torch::tensor({predict_step}));
            auto input_tr = torch::cat(input_vec);
            //std::cout << "input_tr: " << input_tr.sizes() << std::endl;
            ds.data = input_tr;
            //收集label
            auto& label = buff_pre_id.second.labels[label_index];
            std::vector<double> label_vec = {
                label.vx, label.vy, label.vz,
                label.w, label.r1, label.r2
            };
            ds.label = torch::tensor(label_vec);
            datasets.push_back(ds);
            ++start, ++end, ++predict_step_index, ++label_index;
        }
    }
    auto end_time = std::chrono::system_clock::now();
    auto duration = end_time - start_time;
    std::cout << "It cost " << duration.count() << " ms" << std::endl;
    //TODO 传输数据


    state_ = First;
}
