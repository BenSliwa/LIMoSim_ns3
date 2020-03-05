#ifndef LOCALVEHICLEMANAGER_H
#define LOCALVEHICLEMANAGER_H

#include <map>
#include "LIMoSim/world/vector3d.h"
#include "LIMoSim/mobility/mobilitydata.h"

namespace LIMoSim {

class Vehicle;
typedef std::map<std::string, MobilityData> MobilityDataMap;

class LocalVehicleManager
{
public:
    LocalVehicleManager(Vehicle *_vehicle);
    ~LocalVehicleManager();

    const MobilityDataMap& getMobilityData();

    void updateMobilityData();

    MobilityData getVehicleMobiliyData();

    /**
     * @brief updateMobilityDataReport
     * Update the local vehicle manager's knowledge of the mobility
     * situation of the world.
     * @param _mobilitDataReport
     */
    void updateMobilityDataReport(MobilityDataMap _mobilitDataReport);
    void updateMobilityDataReport(MobilityData _mobilitData);

private:
    Vehicle *m_vehicle;
    MobilityDataMap m_mobilityDataReport;
};

} // namespace LIMoSim

#endif // LOCALVEHICLEMANAGER_H
