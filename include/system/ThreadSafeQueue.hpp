// ThreadSafeQueue.hpp - Lock-based thread-safe FIFO queue.
#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>

namespace warehouse {

template<typename T>
class ThreadSafeQueue {
public:
    void push(T value) {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_queue.push(std::move(value));
        }
        m_cv.notify_one();
    }

    /// Blocking pop. Returns std::nullopt if woken up but queue still empty.
    std::optional<T> pop() {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_cv.wait(lock, [this]{ return !m_queue.empty() || m_stop; });
        if (m_queue.empty()) return std::nullopt;
        T val = std::move(m_queue.front());
        m_queue.pop();
        return val;
    }

    /// Non-blocking try_pop.
    std::optional<T> tryPop() {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_queue.empty()) return std::nullopt;
        T val = std::move(m_queue.front());
        m_queue.pop();
        return val;
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_stop = true;
        }
        m_cv.notify_all();
    }

    [[nodiscard]] bool empty() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.empty();
    }

    [[nodiscard]] std::size_t size() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.size();
    }

private:
    mutable std::mutex      m_mutex;
    std::condition_variable m_cv;
    std::queue<T>           m_queue;
    bool                    m_stop{false};
};

}  // namespace warehouse
