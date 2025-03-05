//
// Created by lbw on 25-3-4.
//

#ifndef WRITER_H
#define WRITER_H
#include <torch/torch.h>
#include <filesystem>
#include "nn_MP.h"
#include "logger/logger.h"

class Writer {
public:
    Writer(std::string path);
    ~Writer();

    void init();
    std::unique_ptr<nn_MP> save(std::unique_ptr<nn_MP> net);
private:
    std::string dir_path_;
    std::string private_path_;
    int save_count_ = 0;
};



#endif //WRITER_H
