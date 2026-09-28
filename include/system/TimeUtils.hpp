// TimeUtils.hpp - High-resolution timing utilities using std::chrono.
#pragma once

#include <chrono>
#include <string>

namespace warehouse {

using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

class Timer {
public:
    Timer() : m_start(Clock::now()) {}

    void reset() noexcept { m_start = Clock::now(); }

    [[nodiscard]] double elapsedMs() const noexcept {
        auto dur = Clock::now() - m_start;
        return std::chrono::duration<double, std::milli>(dur).count();
    }

    [[nodiscard]] double elapsedSeconds() const noexcept {
        return elapsedMs() / 1000.0;
    }

private:
    TimePoint m_start;
};

/// Format seconds as "Xm Y.Zs" or "Y.Zs".
[[nodiscard]] inline std::string formatTime(double seconds) {
    int m = static_cast<int>(seconds) / 60;
    double s = seconds - m * 60.0;
    char buf[64];
    if (m > 0)
        snprintf(buf, sizeof(buf), "%dm %.1fs", m, s);
    else
        snprintf(buf, sizeof(buf), "%.1fs", s);
    return buf;
}

}  // namespace warehouse
