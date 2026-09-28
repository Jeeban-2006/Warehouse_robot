/* warehouse_sensor_driver.c
 * Linux virtual character device driver for simulated warehouse robot sensors.
 *
 * Device: /dev/warehouse_sensor
 * Type:   Miscdevice (character device using misc_register)
 *
 * Supported operations:
 *   open()         - Open the device
 *   release()      - Close the device
 *   read()         - Read latest sensor reading as binary struct
 *   write()        - Write sensor reading (simulate sensor update)
 *   unlocked_ioctl - WS_IOC_GET_READING, WS_IOC_SET_READING,
 *                    WS_IOC_RESET, WS_IOC_GET_STATUS
 *   poll()         - Wait for sensor update (POLLIN when data changes)
 *
 * Synchronization:
 *   A single spinlock protects the shared sensor_data structure.
 *   copy_to_user / copy_from_user are used for all user/kernel transfers.
 *
 * Author: Autonomous Warehouse Robot Project
 * Environment: Linux 6.x (WSL2 Ubuntu 24.04 build target)
 * NOTE: This driver is a virtual/simulated sensor. It does not interact
 *       with physical hardware.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/atomic.h>
#include <linux/string.h>

#include "warehouse_sensor_ioctl.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Warehouse Robot Project");
MODULE_DESCRIPTION("Virtual warehouse robot sensor character device driver");
MODULE_VERSION("1.0");

/* ── Module parameters ─────────────────────────────────────────────────────── */
static int obstacle_threshold = 3;  /* Grid cells: trigger obstacle_detected */
module_param(obstacle_threshold, int, 0644);
MODULE_PARM_DESC(obstacle_threshold, "Distance threshold for obstacle_detected flag");

/* ── Driver state ──────────────────────────────────────────────────────────── */

static struct WarehouseSensorReading sensor_data;
static spinlock_t sensor_lock;
static wait_queue_head_t sensor_wq;
static atomic_t data_ready;           /* Non-zero when new data available   */
static atomic_t open_count;           /* Number of open file descriptors    */

/* ── File operations ───────────────────────────────────────────────────────── */

static int ws_open(struct inode *inode, struct file *filp)
{
    atomic_inc(&open_count);
    pr_info("warehouse_sensor: device opened (count=%d)\n",
            atomic_read(&open_count));
    return 0;
}

static int ws_release(struct inode *inode, struct file *filp)
{
    atomic_dec(&open_count);
    pr_info("warehouse_sensor: device released (count=%d)\n",
            atomic_read(&open_count));
    return 0;
}

/**
 * ws_read - Read current sensor data as raw binary struct.
 * User-space receives sizeof(struct WarehouseSensorReading) bytes.
 */
static ssize_t ws_read(struct file *filp, char __user *buf,
                        size_t count, loff_t *ppos)
{
    struct WarehouseSensorReading local_data;
    unsigned long flags;

    if (count < sizeof(local_data))
        return -EINVAL;

    /* Take a consistent snapshot under spinlock */
    spin_lock_irqsave(&sensor_lock, flags);
    local_data = sensor_data;
    spin_unlock_irqrestore(&sensor_lock, flags);

    /* Reset ready flag after read */
    atomic_set(&data_ready, 0);

    if (copy_to_user(buf, &local_data, sizeof(local_data)))
        return -EFAULT;

    return sizeof(local_data);
}

/**
 * ws_write - Accept a sensor reading from user space.
 * The simulator writes updated robot sensor data via this path.
 */
static ssize_t ws_write(struct file *filp, const char __user *buf,
                         size_t count, loff_t *ppos)
{
    struct WarehouseSensorReading new_data;
    unsigned long flags;

    if (count < sizeof(new_data))
        return -EINVAL;

    if (copy_from_user(&new_data, buf, sizeof(new_data)))
        return -EFAULT;

    /* Update obstacle_detected based on threshold */
    new_data.obstacle_detected =
        (new_data.front_distance >= 0 && new_data.front_distance <= obstacle_threshold) ||
        (new_data.rear_distance  >= 0 && new_data.rear_distance  <= obstacle_threshold) ||
        (new_data.left_distance  >= 0 && new_data.left_distance  <= obstacle_threshold) ||
        (new_data.right_distance >= 0 && new_data.right_distance <= obstacle_threshold);

    spin_lock_irqsave(&sensor_lock, flags);
    sensor_data = new_data;
    spin_unlock_irqrestore(&sensor_lock, flags);

    /* Signal waiting readers */
    atomic_set(&data_ready, 1);
    wake_up_interruptible(&sensor_wq);

    pr_debug("warehouse_sensor: write seq=%d front=%d rear=%d left=%d right=%d obs=%d\n",
             new_data.sequence,
             new_data.front_distance, new_data.rear_distance,
             new_data.left_distance,  new_data.right_distance,
             new_data.obstacle_detected);

    return sizeof(new_data);
}

/**
 * ws_ioctl - Handle ioctl commands.
 */
static long ws_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct WarehouseSensorReading local_data;
    unsigned long flags;
    int status;

    /* Verify magic number and command number */
    if (_IOC_TYPE(cmd) != WS_IOC_MAGIC)   return -ENOTTY;
    if (_IOC_NR(cmd)   > WS_IOC_MAXNR)    return -ENOTTY;

    switch (cmd) {

    case WS_IOC_GET_READING:
        spin_lock_irqsave(&sensor_lock, flags);
        local_data = sensor_data;
        spin_unlock_irqrestore(&sensor_lock, flags);
        if (copy_to_user((struct WarehouseSensorReading __user *)arg,
                         &local_data, sizeof(local_data)))
            return -EFAULT;
        pr_debug("warehouse_sensor: ioctl GET_READING seq=%d\n",
                 local_data.sequence);
        return 0;

    case WS_IOC_SET_READING:
        if (copy_from_user(&local_data,
                           (struct WarehouseSensorReading __user *)arg,
                           sizeof(local_data)))
            return -EFAULT;
        local_data.obstacle_detected =
            (local_data.front_distance >= 0 && local_data.front_distance <= obstacle_threshold) ||
            (local_data.rear_distance  >= 0 && local_data.rear_distance  <= obstacle_threshold) ||
            (local_data.left_distance  >= 0 && local_data.left_distance  <= obstacle_threshold) ||
            (local_data.right_distance >= 0 && local_data.right_distance <= obstacle_threshold);
        spin_lock_irqsave(&sensor_lock, flags);
        sensor_data = local_data;
        spin_unlock_irqrestore(&sensor_lock, flags);
        atomic_set(&data_ready, 1);
        wake_up_interruptible(&sensor_wq);
        return 0;

    case WS_IOC_RESET:
        spin_lock_irqsave(&sensor_lock, flags);
        memset(&sensor_data, 0, sizeof(sensor_data));
        sensor_data.front_distance = -1;
        sensor_data.rear_distance  = -1;
        sensor_data.left_distance  = -1;
        sensor_data.right_distance = -1;
        spin_unlock_irqrestore(&sensor_lock, flags);
        atomic_set(&data_ready, 0);
        pr_info("warehouse_sensor: sensor reset\n");
        return 0;

    case WS_IOC_GET_STATUS:
        status = WS_STATUS_ONLINE;
        spin_lock_irqsave(&sensor_lock, flags);
        if (sensor_data.obstacle_detected)
            status |= WS_STATUS_OBSTACLE;
        spin_unlock_irqrestore(&sensor_lock, flags);
        if (copy_to_user((int __user *)arg, &status, sizeof(status)))
            return -EFAULT;
        return 0;

    default:
        return -ENOTTY;
    }
}

/**
 * ws_poll - Allow poll/select on the device.
 * Returns POLLIN | POLLRDNORM when new data is available.
 */
static __poll_t ws_poll(struct file *filp, poll_table *wait)
{
    __poll_t mask = 0;
    poll_wait(filp, &sensor_wq, wait);
    if (atomic_read(&data_ready))
        mask |= POLLIN | POLLRDNORM;
    return mask;
}

/* ── File operations table ─────────────────────────────────────────────────── */
static const struct file_operations ws_fops = {
    .owner          = THIS_MODULE,
    .open           = ws_open,
    .release        = ws_release,
    .read           = ws_read,
    .write          = ws_write,
    .unlocked_ioctl = ws_ioctl,
    .poll           = ws_poll,
};

/* ── Miscdevice registration ───────────────────────────────────────────────── */
static struct miscdevice ws_miscdev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "warehouse_sensor",
    .fops  = &ws_fops,
    .mode  = 0666,   /* rw-rw-rw- so regular users can access without sudo */
};

/* ── Module init / exit ────────────────────────────────────────────────────── */

static int __init ws_init(void)
{
    int ret;

    spin_lock_init(&sensor_lock);
    init_waitqueue_head(&sensor_wq);
    atomic_set(&data_ready, 0);
    atomic_set(&open_count, 0);

    /* Initialize sensor to default "no obstacle" state */
    memset(&sensor_data, 0, sizeof(sensor_data));
    sensor_data.front_distance = -1;
    sensor_data.rear_distance  = -1;
    sensor_data.left_distance  = -1;
    sensor_data.right_distance = -1;

    ret = misc_register(&ws_miscdev);
    if (ret) {
        pr_err("warehouse_sensor: misc_register failed (%d)\n", ret);
        return ret;
    }

    pr_info("warehouse_sensor: driver loaded — /dev/warehouse_sensor ready\n");
    pr_info("warehouse_sensor: obstacle_threshold=%d cells\n", obstacle_threshold);
    return 0;
}

static void __exit ws_exit(void)
{
    misc_deregister(&ws_miscdev);
    pr_info("warehouse_sensor: driver unloaded\n");
}

module_init(ws_init);
module_exit(ws_exit);
