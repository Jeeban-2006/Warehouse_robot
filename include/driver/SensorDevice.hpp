// SensorDevice.hpp - C++ RAII wrapper for /dev/warehouse_sensor.
// Encapsulates all Linux file descriptor operations.
#pragma once

#include "SensorTypes.hpp"
#include <string>
#include <atomic>
#include <thread>
#include <functional>
#include <mutex>

namespace warehouse {

class SensorDevice {
public:
    static constexpr const char* DEFAULT_DEVICE = "/dev/warehouse_sensor";

    /// Callback type invoked when sensor data changes.
    using SensorCallback = std::function<void(const SensorReading&)>;

    explicit SensorDevice(const std::string& devicePath = DEFAULT_DEVICE);
    ~SensorDevice();

    // Non-copyable, non-movable (owns a file descriptor)
    SensorDevice(const SensorDevice&)            = delete;
    SensorDevice& operator=(const SensorDevice&) = delete;

    // ── Device lifecycle ──────────────────────────────────────────────────
    [[nodiscard]] bool open();
    void close();
    [[nodiscard]] bool isOpen() const noexcept { return m_fd >= 0; }

    // ── Data transfer ─────────────────────────────────────────────────────
    [[nodiscard]] bool readSensor(SensorReading& out) const;
    [[nodiscard]] bool writeSensor(const SensorReading& data) const;

    // ── ioctl operations ──────────────────────────────────────────────────
    [[nodiscard]] bool getReading(SensorReading& out) const;
    [[nodiscard]] bool setReading(const SensorReading& data) const;
    [[nodiscard]] bool reset() const;
    [[nodiscard]] bool getStatus(int& status) const;

    // ── Polling / async monitoring ────────────────────────────────────────
    /// Start background thread that polls fd and fires callback on update.
    void startMonitor(SensorCallback cb, int timeoutMs = 100);
    void stopMonitor();

    // ── Last error ────────────────────────────────────────────────────────
    [[nodiscard]] const std::string& lastError() const noexcept { return m_lastError; }

    // ── Device availability check (static) ────────────────────────────────
    [[nodiscard]] static bool deviceExists(const std::string& path = DEFAULT_DEVICE);

private:
    std::string m_devicePath;
    int         m_fd{-1};
    mutable std::string m_lastError;

    // ── Monitor thread ────────────────────────────────────────────────────
    std::thread           m_monitorThread;
    std::atomic<bool>     m_monitorRunning{false};
    SensorCallback        m_callback;
    int                   m_pollTimeoutMs{100};
    mutable std::mutex    m_fdMutex;

    void monitorLoop();

    void setError(const std::string& msg) const;
};

}  // namespace warehouse
