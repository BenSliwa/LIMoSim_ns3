#ifndef CONSUMPTIONEXPORTER_H
#define CONSUMPTIONEXPORTER_H

#include <iostream>
#include <fstream>
#include <map>

#include "consumption.h"

namespace LIMoSim {
namespace Energy {

typedef std::map<std::string, std::ofstream*> FileStreamMap;

class ConsumptionExporter
{
public:
    static ConsumptionExporter* getInstance();
    ~ConsumptionExporter();

    // Registration control
    void registerVehicle(std::string _vehicleId);
    void unregisterVehicle(std::string _vehicleId);

    // Export control
    void exportVehicleConsumption(std::string _vehicleId, Consumption _consumption);

private:
    FileStreamMap m_vehicleFiles;

    ConsumptionExporter();

};

} // namespace Energy
} // namespace LIMoSim

#endif // CONSUMPTIONEXPORTER_H
