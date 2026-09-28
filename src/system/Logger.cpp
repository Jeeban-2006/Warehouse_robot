// Logger.cpp - Thread-safe structured logger.
#include "system/Logger.hpp"
#include <iostream>

namespace warehouse {

Logger& Logger::instance() {
    static Logger inst;
    return inst;
}

Logger::~Logger() {
    flush();
    if (m_file.is_open()) m_file.close();
}

void Logger::init(const std::string& filename, LogLevel minLevel) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_minLevel = minLevel;
    if (!filename.empty()) {
        m_file.open(filename, std::ios::app);
        if (!m_file) {
            std::cerr << "[Logger] Warning: cannot open log file: " << filename << "\n";
        }
    }
    // Write session start marker
    std::string hdr = "=== SESSION START " + timestamp() + " ===\n";
    if (m_file) m_file << hdr;
    std::cout << hdr;
}

void Logger::log(LogLevel level, const std::string& msg) {
    if (level < m_minLevel) return;
    std::string line = "[" + timestamp() + "] [" + levelStr(level) + "] " + msg + "\n";
    std::lock_guard<std::mutex> lock(m_mutex);
    std::cout << line;
    if (m_file) m_file << line;
}

void Logger::flush() {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::cout.flush();
    if (m_file) m_file.flush();
}

const char* Logger::levelStr(LogLevel l) noexcept {
    switch(l) {
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO:  return "INFO ";
        case LogLevel::WARN:  return "WARN ";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::FATAL: return "FATAL";
        default:              return "?????";
    }
}

std::string Logger::timestamp() {
    auto now = std::chrono::system_clock::now();
    auto t   = std::chrono::system_clock::to_time_t(now);
    auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                   now.time_since_epoch()) % 1000;
    std::tm tm_info{};
#ifdef _WIN32
    localtime_s(&tm_info, &t);
#else
    localtime_r(&t, &tm_info);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%H:%M:%S", &tm_info);
    char ms_buf[8];
    std::snprintf(ms_buf, sizeof(ms_buf), ".%03d", static_cast<int>(ms.count()));
    return std::string(buf) + ms_buf;
}

}  // namespace warehouse
