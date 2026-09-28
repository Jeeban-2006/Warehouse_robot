// SensorDevice.cpp - C++ RAII wrapper for /dev/warehouse_sensor.
// Uses Linux file descriptors, ioctl, and poll for async monitoring.
#include "driver/SensorDevice.hpp"
#include "system/Logger.hpp"

// Linux-specific headers — only compiled on Linux
#ifdef __linux__
#  include <fcntl.h>
#  include <unistd.h>
#  include <sys/ioctl.h>
#  include <poll.h>
#  include <cerrno>
#  include <cstring>
// Include the shared ioctl header (path relative to project root)
#  include "../../driver/warehouse_sensor_ioctl.h"
#endif

#include <sys/stat.h>
#include <sstream>
#include <chrono>
#include <thread>

namespace warehouse {

SensorDevice::SensorDevice(const std::string& devicePath)
    : m_devicePath(devicePath) {}

SensorDevice::~SensorDevice() {
    stopMonitor();
    close();
}

bool SensorDevice::open() {
#ifdef __linux__
    std::lock_guard<std::mutex> lock(m_fdMutex);
    if (m_fd >= 0) return true;   // Already open

    m_fd = ::open(m_devicePath.c_str(), O_RDWR | O_CLOEXEC);
    if (m_fd < 0) {
        setError(std::string("open() failed: ") + std::strerror(errno));
        LOG_ERROR("SensorDevice: " + m_lastError);
        return false;
    }
    LOG_INFO("SensorDevice: opened " + m_devicePath + " fd=" + std::to_string(m_fd));
    return true;
#else
    setError("SensorDevice::open() - not on Linux; driver not available.");
    LOG_WARN("SensorDevice: " + m_lastError);
    return false;
#endif
}

void SensorDevice::close() {
#ifdef __linux__
    std::lock_guard<std::mutex> lock(m_fdMutex);
    if (m_fd >= 0) {
        ::close(m_fd);
        LOG_INFO("SensorDevice: closed fd=" + std::to_string(m_fd));
        m_fd = -1;
    }
#endif
}

bool SensorDevice::deviceExists(const std::string& path) {
    struct stat st{};
    return (::stat(path.c_str(), &st) == 0);
}

// ── read() based data transfer ────────────────────────────────────────────────
bool SensorDevice::readSensor(SensorReading& out) const {
#ifdef __linux__
    std::lock_guard<std::mutex> lock(m_fdMutex);
    if (m_fd < 0) { setError("Device not open"); return false; }

    struct WarehouseSensorReading kr{};
    ssize_t n = ::read(m_fd, &kr, sizeof(kr));
    if (n != static_cast<ssize_t>(sizeof(kr))) {
        setError(std::string("read() failed: ") + std::strerror(errno));
        return false;
    }
    out.frontDistance    = kr.front_distance;
    out.rearDistance     = kr.rear_distance;
    out.leftDistance     = kr.left_distance;
    out.rightDistance    = kr.right_distance;
    out.obstacleDetected = kr.obstacle_detected != 0;
    out.sequence         = kr.sequence;
    out.robotCol         = kr.robot_col;
    out.robotRow         = kr.robot_row;
    out.robotState       = kr.robot_state;
    return true;
#else
    (void)out;
    setError("readSensor() - not on Linux");
    return false;
#endif
}

bool SensorDevice::writeSensor(const SensorReading& data) const {
#ifdef __linux__
    std::lock_guard<std::mutex> lock(m_fdMutex);
    if (m_fd < 0) { setError("Device not open"); return false; }

    struct WarehouseSensorReading kr{};
    kr.front_distance    = data.frontDistance;
    kr.rear_distance     = data.rearDistance;
    kr.left_distance     = data.leftDistance;
    kr.right_distance    = data.rightDistance;
    kr.obstacle_detected = data.obstacleDetected ? 1 : 0;
    kr.sequence          = data.sequence;
    kr.robot_col         = data.robotCol;
    kr.robot_row         = data.robotRow;
    kr.robot_state       = data.robotState;

    ssize_t n = ::write(m_fd, &kr, sizeof(kr));
    if (n != static_cast<ssize_t>(sizeof(kr))) {
        setError(std::string("write() failed: ") + std::strerror(errno));
        return false;
    }
    return true;
#else
    (void)data;
    setError("writeSensor() - not on Linux");
    return false;
#endif
}

// ── ioctl operations ──────────────────────────────────────────────────────────
bool SensorDevice::getReading(SensorReading& out) const {
#ifdef __linux__
    std::lock_guard<std::mutex> lock(m_fdMutex);
    if (m_fd < 0) { setError("Device not open"); return false; }

    struct WarehouseSensorReading kr{};
    if (::ioctl(m_fd, WS_IOC_GET_READING, &kr) < 0) {
        setError(std::string("ioctl(GET_READING) failed: ") + std::strerror(errno));
        return false;
    }
    out.frontDistance    = kr.front_distance;
    out.rearDistance     = kr.rear_distance;
    out.leftDistance     = kr.left_distance;
    out.rightDistance    = kr.right_distance;
    out.obstacleDetected = kr.obstacle_detected != 0;
    out.sequence         = kr.sequence;
    out.robotCol         = kr.robot_col;
    out.robotRow         = kr.robot_row;
    out.robotState       = kr.robot_state;
    return true;
#else
    (void)out;
    setError("getReading() - not on Linux");
    return false;
#endif
}

bool SensorDevice::setReading(const SensorReading& data) const {
#ifdef __linux__
    std::lock_guard<std::mutex> lock(m_fdMutex);
    if (m_fd < 0) { setError("Device not open"); return false; }

    struct WarehouseSensorReading kr{};
    kr.front_distance    = data.frontDistance;
    kr.rear_distance     = data.rearDistance;
    kr.left_distance     = data.leftDistance;
    kr.right_distance    = data.rightDistance;
    kr.obstacle_detected = data.obstacleDetected ? 1 : 0;
    kr.sequence          = data.sequence;
    kr.robot_col         = data.robotCol;
    kr.robot_row         = data.robotRow;
    kr.robot_state       = data.robotState;

    if (::ioctl(m_fd, WS_IOC_SET_READING, &kr) < 0) {
        setError(std::string("ioctl(SET_READING) failed: ") + std::strerror(errno));
        return false;
    }
    return true;
#else
    (void)data;
    setError("setReading() - not on Linux");
    return false;
#endif
}

bool SensorDevice::reset() const {
#ifdef __linux__
    std::lock_guard<std::mutex> lock(m_fdMutex);
    if (m_fd < 0) { setError("Device not open"); return false; }
    if (::ioctl(m_fd, WS_IOC_RESET) < 0) {
        setError(std::string("ioctl(RESET) failed: ") + std::strerror(errno));
        return false;
    }
    return true;
#else
    setError("reset() - not on Linux");
    return false;
#endif
}

bool SensorDevice::getStatus(int& status) const {
#ifdef __linux__
    std::lock_guard<std::mutex> lock(m_fdMutex);
    if (m_fd < 0) { setError("Device not open"); return false; }
    if (::ioctl(m_fd, WS_IOC_GET_STATUS, &status) < 0) {
        setError(std::string("ioctl(GET_STATUS) failed: ") + std::strerror(errno));
        return false;
    }
    return true;
#else
    (void)status;
    setError("getStatus() - not on Linux");
    return false;
#endif
}

// ── Poll-based monitor thread ─────────────────────────────────────────────────
void SensorDevice::startMonitor(SensorCallback cb, int timeoutMs) {
    if (m_monitorRunning) return;
    m_callback       = std::move(cb);
    m_pollTimeoutMs  = timeoutMs;
    m_monitorRunning = true;
    m_monitorThread  = std::thread(&SensorDevice::monitorLoop, this);
    LOG_INFO("SensorDevice: monitor thread started");
}

void SensorDevice::stopMonitor() {
    m_monitorRunning = false;
    if (m_monitorThread.joinable()) {
        m_monitorThread.join();
        LOG_INFO("SensorDevice: monitor thread stopped");
    }
}

void SensorDevice::monitorLoop() {
#ifdef __linux__
    while (m_monitorRunning) {
        int fd_local;
        {
            std::lock_guard<std::mutex> lock(m_fdMutex);
            fd_local = m_fd;
        }
        if (fd_local < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // Use poll() to wait for POLLIN (data ready)
        struct pollfd pfd{fd_local, POLLIN, 0};
        int ret = ::poll(&pfd, 1, m_pollTimeoutMs);
        if (ret > 0 && (pfd.revents & POLLIN)) {
            SensorReading sr;
            if (readSensor(sr) && m_callback)
                m_callback(sr);
        } else if (ret < 0 && errno != EINTR) {
            LOG_ERROR("SensorDevice: poll() error: " + std::string(std::strerror(errno)));
            break;
        }
    }
#else
    // Fallback: no poll on non-Linux — just sleep
    while (m_monitorRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds(m_pollTimeoutMs));
    }
#endif
}

void SensorDevice::setError(const std::string& msg) const {
    m_lastError = msg;
}

}  // namespace warehouse
