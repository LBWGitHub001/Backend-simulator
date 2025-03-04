//
// Created by lbw on 25-3-4.
//

#ifndef LOGGER_H
#define LOGGER_H
#include <mutex>
#include <unordered_map>
#define LOG_DEBUG(...) Logger::Log(LogLevel::DEBUG, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_INFO(...)  Logger::Log(LogLevel::INFO,  __FILE__, __LINE__, __VA_ARGS__)
#define LOG_WARN(...)  Logger::Log(LogLevel::WARN,  __FILE__, __LINE__, __VA_ARGS__)
#define LOG_ERROR(...) Logger::Log(LogLevel::ERROR, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_FATAL(...) Logger::Log(LogLevel::FATAL, __FILE__, __LINE__, __VA_ARGS__)

#define PUT_DEBUG(INPUT) LOG(LogLevel::DEBUG, INPUT)
#define PUT_INFO(INPUT) LOG(LogLevel::INFO,  INPUT)
#define PUT_WARN(INPUT) LOG(LogLevel::WARN,  INPUT)
#define PUT_ERROR(INPUT) LOG(LogLevel::ERROR, INPUT)
#define PUT_FATAL(INPUT) LOG(LogLevel::FATAL, INPUT)

#define LOG(level,INPUT) \
    {\
std::cerr << Logger::levelColors.at(level);\
std::cerr << INPUT << Logger::RESET;\
}


enum class LogLevel
{
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL
};

class Logger
{
public:
    static Logger& getLogger();
    static void Log(LogLevel level, const char* file, int line, const char* format, ...);
    static void put(LogLevel level, const char* format, ...);

private:
    Logger();
    ~Logger();
    static Logger* logger;

public:
    static const std::unordered_map<LogLevel, std::string> levelColors;
    static const std::string RESET;
    static std::mutex outputMutex_;
};


#endif //LOGGER_H
