# System Programming Design
Thread Model: Main (UI), Simulation (Logic), Sensor (Poll).
Sync: std::mutex, atomics, condition_variable.
IPC: Kernel module reads.
