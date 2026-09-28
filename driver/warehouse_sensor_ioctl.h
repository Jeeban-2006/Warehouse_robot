/* warehouse_sensor_ioctl.h
 * Shared header between kernel driver and C++ user-space application.
 * Defines ioctl commands and the sensor data structure.
 *
 * This header MUST be valid both as C (for kernel) and as C++ (for user-space).
 * Include it from both warehouse_sensor_driver.c and SensorDevice.cpp.
 */
#ifndef WAREHOUSE_SENSOR_IOCTL_H
#define WAREHOUSE_SENSOR_IOCTL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/ioctl.h>

/* ------------------------------------------------------------------
 * Magic number (unique to this driver — chosen to avoid collisions).
 * Using 'W' (0x57).
 * ------------------------------------------------------------------ */
#define WS_IOC_MAGIC  'W'

/* ------------------------------------------------------------------
 * Sensor reading structure.
 *
 * All distances are in grid cells.
 * -1 means "no obstacle detected up to grid boundary".
 * ------------------------------------------------------------------ */
struct WarehouseSensorReading {
    int front_distance;   /* cells to nearest obstacle ahead   */
    int rear_distance;    /* cells to nearest obstacle behind  */
    int left_distance;    /* cells to nearest obstacle left    */
    int right_distance;   /* cells to nearest obstacle right   */
    int obstacle_detected;/* non-zero if any distance <= threshold */
    int sequence;         /* monotonically increasing update counter */
    int robot_col;        /* current robot column                   */
    int robot_row;        /* current robot row                      */
    int robot_state;      /* RobotState enum value                  */
    int pad;              /* explicit padding for alignment          */
};

/* ------------------------------------------------------------------
 * IOCTL command definitions
 *
 * _IOR  = read from device (kernel → user)
 * _IOW  = write to device  (user  → kernel)
 * _IOWR = read + write
 * ------------------------------------------------------------------ */

/** WS_IOC_GET_READING
 *  Read the latest sensor reading.
 *  Direction: kernel → user (read).
 *  Argument:  pointer to struct WarehouseSensorReading
 */
#define WS_IOC_GET_READING  _IOR(WS_IOC_MAGIC, 1, struct WarehouseSensorReading)

/** WS_IOC_SET_READING
 *  Write (simulate) a sensor reading from user space.
 *  Direction: user → kernel (write).
 *  Argument:  pointer to struct WarehouseSensorReading
 */
#define WS_IOC_SET_READING  _IOW(WS_IOC_MAGIC, 2, struct WarehouseSensorReading)

/** WS_IOC_RESET
 *  Reset the sensor to its default state.
 *  Direction: none (no argument).
 */
#define WS_IOC_RESET        _IO(WS_IOC_MAGIC,  3)

/** WS_IOC_GET_STATUS
 *  Read driver status flags.
 *  Direction: kernel → user.
 *  Argument:  pointer to int
 */
#define WS_IOC_GET_STATUS   _IOR(WS_IOC_MAGIC, 4, int)

/* Status flag bits */
#define WS_STATUS_ONLINE    (1 << 0)   /* driver is active */
#define WS_STATUS_OBSTACLE  (1 << 1)   /* obstacle within threshold */

/* Maximum valid ioctl number for boundary check */
#define WS_IOC_MAXNR        4

#ifdef __cplusplus
}
#endif

#endif /* WAREHOUSE_SENSOR_IOCTL_H */
