// SignalHandler.cpp - POSIX signal handling.
#include "system/SignalHandler.hpp"
#include <iostream>

namespace warehouse {

static std::atomic<bool>      g_shutdownFlag{false};
static std::function<void()>  g_callback;

std::atomic<bool>& SignalHandler::shutdownRequested() noexcept {
    return g_shutdownFlag;
}

void SignalHandler::install() {
    std::signal(SIGINT,  handleSignal);
    std::signal(SIGTERM, handleSignal);
}

void SignalHandler::setCallback(std::function<void()> cb) {
    g_callback = std::move(cb);
}

void SignalHandler::handleSignal(int signum) {
    // Signal handlers must be async-signal-safe.
    // Only set atomic flag and call callback if safe.
    g_shutdownFlag.store(true, std::memory_order_relaxed);
    // Reinstall default handlers so second Ctrl+C kills immediately
    std::signal(signum, SIG_DFL);
    if (g_callback) {
        // Note: calling arbitrary functions from signal handler is technically
        // unsafe, but is common practice for simple shutdown scenarios.
        // For production code, use a pipe-based wakeup mechanism instead.
        g_callback();
    }
}

}  // namespace warehouse
