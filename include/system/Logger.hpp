// Logger.hpp - Thread-safe structured logger with syslog-style levels.
#pragma once

#include <string>
#include <fstream>
#include <mutex>
#include <atomic>
#include <sstream>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <ctime>

namespace warehouse {

enum class LogLevel { DEBUG, INFO, WARN, ERROR, FATAL };

class Logger {
public:
    static Logger& instance();

    void init(const std::string& filename, LogLevel minLevel = LogLevel::INFO);
    void log(LogLevel level, const std::string& msg);

    void debug(const std::string& m) { log(LogLevel::DEBUG, m); }
    void info (const std::string& m) { log(LogLevel::INFO,  m); }
    void warn (const std::string& m) { log(LogLevel::WARN,  m); }
    void error(const std::string& m) { log(LogLevel::ERROR, m); }
    void fatal(const std::string& m) { log(LogLevel::FATAL, m); }

    void setLevel(LogLevel l) noexcept { m_minLevel = l; }
    void flush();

private:
    Logger() = default;
    ~Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    std::ofstream m_file;
    std::mutex    m_mutex;
    LogLevel      m_minLevel{LogLevel::INFO};

    static const char* levelStr(LogLevel l) noexcept;
    static std::string timestamp();
};

// Convenience macros
#define LOG_DEBUG(msg) warehouse::Logger::instance().debug(msg)
#define LOG_INFO(msg)  warehouse::Logger::instance().info(msg)
#define LOG_WARN(msg)  warehouse::Logger::instance().warn(msg)
#define LOG_ERROR(msg) warehouse::Logger::instance().error(msg)
#define LOG_FATAL(msg) warehouse::Logger::instance().fatal(msg)

}  // namespace warehouse
