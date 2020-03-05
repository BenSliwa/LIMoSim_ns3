#ifndef ENERGYMONITOR_H
#define ENERGYMONITOR_H

#include <map>

#include "consumption.h"

namespace LIMoSim {
namespace Energy {

class EnergyMonitor
{
public:
    static EnergyMonitor* getInstance();

    // Monitoring control
    bool isMonitored(std::string _vehicleId);
    void startMonitoring(std::string _vehicleId);
    void stopMonitoring(std::string _vehicleId);

    // Energy data provisioning
    void saveIncrementalUpdate(std::string _vehicleId, double _timeDelta_s, double _energy_J);

private:
    std::map<std::string, Consumption> m_energyConsumption;

    bool exportToFile = true;

    EnergyMonitor();
};

} // namespace Energy
} // namespace LIMoSim

#endif // ENERGYMONITOR_H
