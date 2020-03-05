#ifndef UIDATA_H
#define UIDATA_H

namespace LIMoSim {

class Vehicle;
/**
 * @brief The UIData class
 * Interface for handling core based ui data
 */
class UIData {

public:
    virtual void registerVehicles() = 0;
    virtual void registerVehicle(Vehicle *_vehicle) = 0;
    virtual void unregisterVehicle(std::string _vehicleId) = 0;

};
}

#endif // UIDATA_H
