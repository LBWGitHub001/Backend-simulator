//
// Created by lbw on 25-3-4.
//

#include "logger/logger.h"
#include <iomanip>
#include <cstdarg>
#include <ctime>
#include <iostream>
#include <cstdio>

Logger* Logger::logger = nullptr;
const std::string Logger::RESET = "\033[0m";
const std::unordered_map<LogLevel, std::string> Logger::levelColors = {
    {LogLevel::DEBUG, "\033[1;32m"},  // 亮绿色
    {LogLevel::INFO,  "\033[1;34m"},  // 亮蓝色
    {LogLevel::WARN,  "\033[1;33m"},  // 亮黄色
    {LogLevel::ERROR, "\033[1;31m"},  // 亮红色
    {LogLevel::FATAL, "\033[1;41;37m"} // 白字红底
};
std::mutex Logger::outputMutex_;

Logger& Logger::getLogger()
{
    if (logger == nullptr)
    {
        logger = new Logger();
    }
    return *logger;
}

Logger::Logger() = default;

Logger::~Logger() = default;

void Logger::Log(LogLevel level, const char* file, int line, const char* format, ...) {
    std::unique_lock<std::mutex> lock(outputMutex_);
    // 时间戳
    auto now = std::time(nullptr);

    // 颜色和日志级别
    std::cerr << levelColors.at(level);
    switch (level) {
    case LogLevel::DEBUG: std::cerr << "[DEBUG] "; break;
    case LogLevel::INFO:  std::cerr << "[INFO]  "; break;
    case LogLevel::WARN:  std::cerr << "[WARN]  "; break;
    case LogLevel::ERROR: std::cerr << "[ERROR] "; break;
    case LogLevel::FATAL: std::cerr << "[FATAL] "; break;
    }
    std::tm tm = *std::localtime(&now);
    std::cerr << std::put_time(&tm, "[%H:%M:%S] ");

    // 日志内容
    va_list args;
    va_start(args, format);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    std::cerr << buffer << RESET;
    // 文件名和行号（灰色）
    std::cerr << "\t \033[1;30m" << file << ":" << line << RESET << std::endl;
}

void Logger::put(LogLevel level, const char* format, ...)
{
    // 日志内容
    va_list args;
    va_start(args, format);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    std::cerr << levelColors.at(level) << buffer << RESET;
}
