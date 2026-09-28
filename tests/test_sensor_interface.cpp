// test_sensor_interface.cpp - Tests for SensorDevice C++ interface.
// Note: driver tests require /dev/warehouse_sensor (kernel module loaded on Linux).
// Tests that can run without the driver are marked [NO_DRIVER_REQUIRED].
// Tests that require the driver are marked [REQUIRES_DRIVER].
#include "driver/SensorDevice.hpp"
#include <cassert>
#include <iostream>
#include <string>

namespace {
int passed = 0, failed = 0;

void test(bool cond, const std::string& name) {
    if (cond) { std::cout << "  [PASS] " << name << "\n"; ++passed; }
    else       { std::cout << "  [FAIL] " << name << "\n"; ++failed; }
}

void test_device_not_found() {
    // Test device not found (should fail gracefully)
    warehouse::SensorDevice dev("/dev/nonexistent_device_xyz");
    bool ok = dev.open();
    test(!ok, "Sensor [NO_DRIVER]: open nonexistent device fails gracefully");
    test(!dev.isOpen(), "Sensor [NO_DRIVER]: isOpen() = false");
    test(!dev.lastError().empty(), "Sensor [NO_DRIVER]: error message set");
}

void test_exists_check() {
    bool exists = warehouse::SensorDevice::deviceExists("/dev/nonexistent_abc");
    test(!exists, "Sensor [NO_DRIVER]: deviceExists returns false for nonexistent path");
}

void test_sensor_reading_type() {
    warehouse::SensorReading sr;
    test(sr.frontDistance == -1, "SensorReading: default frontDistance = -1");
    test(sr.rearDistance  == -1, "SensorReading: default rearDistance = -1");
    test(!sr.obstacleDetected,   "SensorReading: default obstacleDetected = false");
}

void test_driver_operations() {
    // These tests require /dev/warehouse_sensor to be present
    bool exists = warehouse::SensorDevice::deviceExists();
    if (!exists) {
        std::cout << "  [SKIP] Driver tests: /dev/warehouse_sensor not available\n";
        std::cout << "         To test: load warehouse_sensor_driver.ko first\n";
        std::cout << "         sudo insmod driver/warehouse_sensor_driver.ko\n";
        // Record as not-tested (don't count as failures)
        return;
    }

    std::cout << "  [INFO] Driver found! Running driver tests.\n";
    warehouse::SensorDevice dev;
    bool opened = dev.open();
    test(opened, "Sensor [REQUIRES_DRIVER]: open /dev/warehouse_sensor");

    if (opened) {
        // Reset
        bool resetOk = dev.reset();
        test(resetOk, "Sensor [REQUIRES_DRIVER]: ioctl RESET");

        // Read via ioctl
        warehouse::SensorReading sr;
        bool getOk = dev.getReading(sr);
        test(getOk, "Sensor [REQUIRES_DRIVER]: ioctl GET_READING");
        if (getOk) {
            test(sr.sequence >= 0, "Sensor [REQUIRES_DRIVER]: sequence >= 0 after reset");
        }

        // Write via ioctl
        warehouse::SensorReading wr;
        wr.frontDistance = 3;
        wr.rearDistance  = 5;
        wr.leftDistance  = 2;
        wr.rightDistance = 7;
        wr.sequence      = 42;
        bool setOk = dev.setReading(wr);
        test(setOk, "Sensor [REQUIRES_DRIVER]: ioctl SET_READING");

        // Read back
        warehouse::SensorReading rr;
        dev.getReading(rr);
        test(rr.frontDistance == 3, "Sensor [REQUIRES_DRIVER]: readback front = 3");
        test(rr.rearDistance  == 5, "Sensor [REQUIRES_DRIVER]: readback rear = 5");

        // Status
        int status = 0;
        bool statusOk = dev.getStatus(status);
        test(statusOk, "Sensor [REQUIRES_DRIVER]: ioctl GET_STATUS");
        test((status & 0x1) != 0, "Sensor [REQUIRES_DRIVER]: STATUS_ONLINE bit set");

        // Read via read()
        warehouse::SensorReading rd;
        bool readOk = dev.readSensor(rd);
        test(readOk, "Sensor [REQUIRES_DRIVER]: read() succeeds");

        dev.close();
        test(!dev.isOpen(), "Sensor [REQUIRES_DRIVER]: closed after close()");
    }
}

}  // anonymous namespace

int main() {
    std::cout << "=== Sensor Device Interface Tests ===\n";
    std::cout << "NOTE: Some tests require kernel module loaded.\n\n";
    test_device_not_found();
    test_exists_check();
    test_sensor_reading_type();
    test_driver_operations();

    std::cout << "\n=== Results: " << passed << " passed, " << failed << " failed ===\n";
    if (failed == 0)
        std::cout << "(Driver-dependent tests may have been skipped — see above)\n";
    return (failed == 0) ? 0 : 1;
}
