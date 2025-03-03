//
// Created by lbw on 25-3-3.
//

#include "memory.h"
Memory::Memory(nn::TrainParam train_param)
{
    config_ = train_param;
}

Memory::~Memory()
{

}

void Memory::registerTrainer(std::unique_ptr<TrainBase> trainer)
{
    trainer_ = std::move(trainer);
}

void Memory::push(interfaces::msg::Armors armors)
{
    auto timestamp =armors.header.stamp.nanosec / 1000.0;


}
