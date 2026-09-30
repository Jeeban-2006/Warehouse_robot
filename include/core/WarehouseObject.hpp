#pragma once
#include <string>
#include <utility>
#include "Cell.hpp"

namespace warehouse {

class WarehouseObject {
public:
    WarehouseObject(int c, int r, std::string n) 
        : col(c), row(r), name(std::move(n)) {}
    virtual ~WarehouseObject() = default;
    
    virtual CellType getType() const = 0;
    
    int col;
    int row;
    std::string name;
};

class Shelf : public WarehouseObject {
public:
    Shelf(int c, int r, std::string n) : WarehouseObject(c, r, std::move(n)) {}
    CellType getType() const override { return CellType::SHELF; }
};

class ChargingStation : public WarehouseObject {
public:
    ChargingStation(int c, int r, std::string n) : WarehouseObject(c, r, std::move(n)) {}
    CellType getType() const override { return CellType::CHARGING_STATION; }
};

class LoadingZone : public WarehouseObject {
public:
    LoadingZone(int c, int r, std::string n) : WarehouseObject(c, r, std::move(n)) {}
    CellType getType() const override { return CellType::LOADING_ZONE; }
};

class PickupStation : public WarehouseObject {
public:
    PickupStation(int c, int r, std::string n) : WarehouseObject(c, r, std::move(n)) {}
    CellType getType() const override { return CellType::PICKUP_STATION; }
};

} // namespace warehouse
