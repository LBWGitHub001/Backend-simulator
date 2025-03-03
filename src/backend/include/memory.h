//
// Created by lbw on 25-3-3.
//

#ifndef MEMORY_H
#define MEMORY_H
#include <rclcpp/rclcpp.hpp>
#include <torch/torch.h>
#include <memory>
#include <interfaces/msg/armors.hpp>

#include "nnTrain/nn_common.h"
#include "nnTrain/train_base.h"

class Memory
{
public:
  Memory(nn::TrainParam train_param);
  ~Memory();

  void registerTrainer(std::unique_ptr<TrainBase> trainer);
  void push(interfaces::msg::Armors armors);

private:
  nn::TrainParam config_;
  std::unique_ptr<TrainBase> trainer_{nullptr};
};


#endif //MEMORY_H
