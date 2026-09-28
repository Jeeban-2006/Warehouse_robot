// sensor_test.cpp - Command-line utility to test /dev/warehouse_sensor.
// Demonstrates: device open, ioctl, read, write, status, reset, close.
#include "driver/SensorDevice.hpp"
#include <iostream>
#include <iomanip>
#include <string>
#include <chrono>
#include <thread>

using namespace warehouse;

static void printReading(const SensorReading& sr) {
    std::cout << "  Front:    " << (sr.frontDistance < 0 ? "clear" : std::to_string(sr.frontDistance) + " cells") << "\n";
    std::cout << "  Rear:     " << (sr.rearDistance  < 0 ? "clear" : std::to_string(sr.rearDistance)  + " cells") << "\n";
    std::cout << "  Left:     " << (sr.leftDistance  < 0 ? "clear" : std::to_string(sr.leftDistance)  + " cells") << "\n";
    std::cout << "  Right:    " << (sr.rightDistance < 0 ? "clear" : std::to_string(sr.rightDistance) + " cells") << "\n";
    std::cout << "  Obstacle: " << (sr.obstacleDetected ? "YES" : "NO") << "\n";
    std::cout << "  Seq:      " << sr.sequence << "\n";
    std::cout << "  Robot:    (" << sr.robotCol << "," << sr.robotRow << ")\n";
}

int main() {
    std::cout << "==========================================\n";
    std::cout << "  Warehouse Sensor Device Test Utility\n";
    std::cout << "==========================================\n\n";
    std::cout << "Device: " << SensorDevice::DEFAULT_DEVICE << "\n\n";

    // Check device existence
    if (!SensorDevice::deviceExists()) {
        std::cout << "[FAIL] Device not found: " << SensorDevice::DEFAULT_DEVICE << "\n";
        std::cout << "\nTo load the driver:\n";
        std::cout << "  cd driver/\n";
        std::cout << "  make\n";
        std::cout << "  sudo insmod warehouse_sensor_driver.ko\n";
        std::cout << "  ls -l /dev/warehouse_sensor\n";
        return 1;
    }
    std::cout << "[OK] Device found.\n\n";

    SensorDevice dev;

    // ── Test 1: Open ──────────────────────────────────────────────────────
    std::cout << "Test 1: Open device\n";
    if (!dev.open()) {
        std::cout << "[FAIL] " << dev.lastError() << "\n";
        return 1;
    }
    std::cout << "  [PASS] Device opened.\n\n";

    // ── Test 2: Reset ────────────────────────────────────────────────────
    std::cout << "Test 2: Reset\n";
    if (dev.reset()) {
        std::cout << "  [PASS] Reset OK.\n\n";
    } else {
        std::cout << "  [FAIL] " << dev.lastError() << "\n";
    }

    // ── Test 3: Read initial state ────────────────────────────────────────
    std::cout << "Test 3: Read initial sensor state (after reset)\n";
    SensorReading initial;
    if (dev.getReading(initial)) {
        std::cout << "  [PASS] ioctl GET_READING:\n";
        printReading(initial);
    } else {
        std::cout << "  [FAIL] " << dev.lastError() << "\n";
    }
    std::cout << "\n";

    // ── Test 4: Write sensor data ─────────────────────────────────────────
    std::cout << "Test 4: Write sensor data (simulate obstacle 3 cells ahead)\n";
    SensorReading wr;
    wr.frontDistance = 3;
    wr.rearDistance  = 9;
    wr.leftDistance  = 2;
    wr.rightDistance = 7;
    wr.sequence      = 100;
    wr.robotCol      = 5;
    wr.robotRow      = 10;
    wr.robotState    = 2;  // MOVING

    if (dev.setReading(wr)) {
        std::cout << "  [PASS] Data written.\n\n";
    } else {
        std::cout << "  [FAIL] " << dev.lastError() << "\n\n";
    }

    // ── Test 5: Read back ─────────────────────────────────────────────────
    std::cout << "Test 5: Read back (verify written data)\n";
    SensorReading rb;
    if (dev.getReading(rb)) {
        std::cout << "  [PASS] ioctl GET_READING:\n";
        printReading(rb);
        bool ok = (rb.frontDistance == 3 && rb.rearDistance == 9);
        std::cout << "  Verify: " << (ok ? "[PASS]" : "[FAIL]") << " front=3, rear=9\n";
    } else {
        std::cout << "  [FAIL] " << dev.lastError() << "\n";
    }
    std::cout << "\n";

    // ── Test 6: Status ────────────────────────────────────────────────────
    std::cout << "Test 6: Get status\n";
    int status = 0;
    if (dev.getStatus(status)) {
        std::cout << "  [PASS] Status: " << status << "\n";
        std::cout << "         ONLINE: "   << ((status & 1) ? "YES" : "NO") << "\n";
        std::cout << "         OBSTACLE: " << ((status & 2) ? "YES" : "NO") << "\n";
    } else {
        std::cout << "  [FAIL] " << dev.lastError() << "\n";
    }
    std::cout << "\n";

    // ── Test 7: Read via read() ───────────────────────────────────────────
    std::cout << "Test 7: Read via read() syscall\n";
    SensorReading rd;
    if (dev.readSensor(rd)) {
        std::cout << "  [PASS] read() OK. front=" << rd.frontDistance << "\n\n";
    } else {
        std::cout << "  [FAIL] " << dev.lastError() << "\n\n";
    }

    // ── Test 8: Write via write() ─────────────────────────────────────────
    std::cout << "Test 8: Write via write() syscall\n";
    SensorReading wd;
    wd.frontDistance = 1;
    wd.sequence      = 200;
    if (dev.writeSensor(wd)) {
        std::cout << "  [PASS] write() OK.\n\n";
    } else {
        std::cout << "  [FAIL] " << dev.lastError() << "\n\n";
    }

    // ── Test 9: Close ─────────────────────────────────────────────────────
    std::cout << "Test 9: Close device\n";
    dev.close();
    std::cout << "  [PASS] Device closed. isOpen=" << dev.isOpen() << "\n\n";

    // ── Test 10: Operation after close (should fail) ──────────────────────
    std::cout << "Test 10: Operation after close (error handling)\n";
    SensorReading dummy;
    bool failOk = !dev.getReading(dummy);
    std::cout << "  " << (failOk ? "[PASS]" : "[FAIL]") << " getReading fails when closed.\n\n";

    std::cout << "==========================================\n";
    std::cout << "  Warehouse Sensor Test: COMPLETE\n";
    std::cout << "==========================================\n";
    return 0;
}
