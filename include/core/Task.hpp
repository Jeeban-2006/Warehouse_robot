#pragma once
#include <string>
#include <utility>
#include <atomic>

namespace warehouse {

enum class TaskStatus {
    CREATED,
    ASSIGNED,
    PICKING,
    DELIVERING,
    COMPLETED,
    FAILED
};

struct Task {
    int id;
    std::pair<int,int> pickupLocation;
    std::pair<int,int> deliveryLocation;
    int priority;
    TaskStatus status;
    std::string itemName;
    
    Task(int i, std::pair<int,int> pick, std::pair<int,int> drop, int p, std::string item)
        : id(i), pickupLocation(pick), deliveryLocation(drop), priority(p), status(TaskStatus::CREATED), itemName(std::move(item)) {}
};

} // namespace warehouse
