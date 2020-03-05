#include "energymonitor.h"
#include <iostream>

#include "consumptionexporter.h"

namespace LIMoSim {
namespace Energy {

EnergyMonitor *EnergyMonitor::getInstance()
{
    static EnergyMonitor instance;
    return &instance;
}

bool EnergyMonitor::isMonitored(std::string _vehicleId)
{
    return m_energyConsumption.count(_vehicleId);
}

void EnergyMonitor::startMonitoring(std::string _vehicleId)
{
    if (isMonitored(_vehicleId)) {
        return;
    }
    m_energyConsumption[_vehicleId] = Consumption();
    ConsumptionExporter::getInstance()->registerVehicle(_vehicleId);
    saveIncrementalUpdate(_vehicleId, 0.0, 0.0);
}

void EnergyMonitor::stopMonitoring(std::string _vehicleId)
{
    if (isMonitored(_vehicleId)) {
        ConsumptionExporter::getInstance()->unregisterVehicle(_vehicleId);
        m_energyConsumption.erase(m_energyConsumption.find(_vehicleId));
    }
}

void EnergyMonitor::saveIncrementalUpdate(std::string _vehicleId, double _timeDelta_s, double _energy_J)
{
    if (isMonitored(_vehicleId)){
        m_energyConsumption.at(_vehicleId).increment(_timeDelta_s, _energy_J);
        ConsumptionExporter::getInstance()->exportVehicleConsumption(_vehicleId, m_energyConsumption.at(_vehicleId));

        // std::cout << "energy increment vehicle " << _vehicleId << " timestep: " << m_energyConsumption.at(_vehicleId).getTimestamp() << " energy: " << m_energyConsumption.at(_vehicleId).getEnergy() << std::endl;
    }
}

EnergyMonitor::EnergyMonitor()
{
    if (exportToFile) {ConsumptionExporter::getInstance();}
}

} // namespace Energy
} // namespace LIMoSim
