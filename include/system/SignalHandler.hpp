// SignalHandler.hpp - POSIX signal handling for graceful shutdown.
// Uses atomic flag to avoid unsafe operations in signal handlers.
#pragma once

#include <atomic>
#include <functional>
#include <csignal>

namespace warehouse {

class SignalHandler {
public:
    /// Returns reference to the global shutdown flag.
    static std::atomic<bool>& shutdownRequested() noexcept;

    /// Install SIGINT and SIGTERM handlers.
    static void install();

    /// Set a callback to invoke when shutdown is triggered.
    static void setCallback(std::function<void()> cb);

private:
    static void handleSignal(int signum);
};

}  // namespace warehouse
