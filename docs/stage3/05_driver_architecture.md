# Driver Architecture
Module: warehouse_sensor.ko
File ops: open, read, release, poll.
Uses mutex for concurrency. copy_to_user to send simulated sensor events to user space.
