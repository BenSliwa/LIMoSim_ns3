#include "localvehiclemanager.h"
#include "LIMoSim/mobility/vehicle.h"
#include "LIMoSim/world/vehiclemanager.h"

namespace LIMoSim {

LocalVehicleManager::LocalVehicleManager(Vehicle* _vehicle):
    m_vehicle(_vehicle)
{
}

LocalVehicleManager::~LocalVehicleManager()
{
}

const MobilityDataMap &LocalVehicleManager::getMobilityData()
{
    return m_mobilityDataReport;
}

void LocalVehicleManager::updateMobilityData()
{
    VehicleManager::getInstance()->updateMobilityReport(
                m_vehicle->getId(),
                MobilityData(
                    m_vehicle->getPosition(),
                    m_vehicle->getOrientation(),
                    m_vehicle->getVelocity(),
                    m_vehicle->getOrientationVelocity(),
                    m_vehicle->getAcceleration(),
                    m_vehicle->getType() + m_vehicle->getId()
                    )
                );
}

MobilityData LocalVehicleManager::getVehicleMobiliyData()
{
    return MobilityData(
                m_vehicle->getPosition(),
                m_vehicle->getOrientation(),
                m_vehicle->getVelocity(),
                m_vehicle->getOrientationVelocity(),
                m_vehicle->getAcceleration(),
                m_vehicle->getId()
                );
}

void LocalVehicleManager::updateMobilityDataReport(const MobilityDataMap _mobilitDataReport)
{
    for (auto report : _mobilitDataReport) {
        // only save reports of other vehicles
        if (report.first != m_vehicle->getId()){
            m_mobilityDataReport[report.first] = report.second;
        }
    }
}

void LocalVehicleManager::updateMobilityDataReport(MobilityData _mobilitData)
{
    if (_mobilitData.name != m_vehicle->getId()) {
        m_mobilityDataReport[_mobilitData.name] = _mobilitData;
    }
}

} // namespace LIMoSim
