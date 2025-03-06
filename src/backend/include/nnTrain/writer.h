//
// Created by lbw on 25-3-4.
//

#ifndef WRITER_H
#define WRITER_H
#include <torch/torch.h>
#include <filesystem>
#include "nn_MP.h"
#include "logger/logger.h"
#include <fstream>

class Writer {
public:
    Writer(std::string path);
    ~Writer();
    void init();
    std::unique_ptr<nn_MP> save(std::unique_ptr<nn_MP> net);
    Writer& operator<<(std::string& str);
    Writer& operator<<(const int& str);
    Writer& operator<<(const float& str);
    Writer& operator<<(const double& str);
    Writer& operator<<(const char* str);
private:
    std::string dir_path_;
    std::string private_path_;
    int save_count_ = 0;

    std::vector<std::string> outputs_;
};



#endif //WRITER_H
