//
// Created by lbw on 25-3-3.
//

#ifndef MEMORY_H
#define MEMORY_H
#include <rclcpp/rclcpp.hpp>
#include <torch/torch.h>
#include <memory>
#include <thread>
#include <map>
#include <interfaces/msg/armors.hpp>
#include "logger/logger.h"
#include "nnTrain/nn_common.h"
#include "nnTrain/train_base.h"
#include "nnTrain/MPTrainer.h"
#include "interfaces/msg/label.hpp"

class Memory
{
  enum SelectState
  {
    First,      /*!当前识别到的是该系列机器人的第一帧*/
    Selecting,  /*!当前正在收集数据，当前帧和上一帧的机器人类型相同*/
    Lost,       /*!目标丢失，当前帧和上一帧的类型不同，将自动转换为Uploading*/
    Uploading,   /*!已经收集完成数据，准备上传*/
  };

public:
  Memory(const nn::TrainParam& train_param);
  ~Memory();

  void setNet(std::unique_ptr<torch::nn::Module> net);
  void initBuff();
  void registerTrainer(std::unique_ptr<MPTrainer> trainer);
  void push(interfaces::msg::Armors armors);

private:
  // State
  SelectState state_;
  double start_time_;
  std::string last_type_;
  nn::lazyDataSet input_buffer_;
  std::map<std::string,int> type_count_;

  // Train
  nn::TrainParam config_;
  void upload(std::string type);
  std::unique_ptr<MPTrainer> trainer_{nullptr};
};


#endif //MEMORY_H
