#include "behavior_separation.h"
#include "behavior_noop.h"
#include "behavior_collisionavoidance.h"

#include "LIMoSim/mobility/uav/uav.h"
#include "LIMoSim/world/vehiclemanager.h"

namespace LIMoSim {

Behavior_Separation::Behavior_Separation(double _searchRadius, std::string _agent):
    Behavior("Separation", _agent),
    m_searchRadius(_searchRadius)
{

}

Steering Behavior_Separation::apply()
{
    MobilityDataMap nearbyUAVs = findNearByUAVs();
    Vector3d repulsiveForce;

    if (!nearbyUAVs.size()) {
        return Steering(Orientation3d(), (getAgent()->getVelocity() * -1) -getAgent()->getVelocity());
    }

    for (auto it = nearbyUAVs.begin(); it != nearbyUAVs.end(); it++) {
        MobilityData data = it->second;
        Vector3d distanceVec = getAgent()->getPosition() - data.position;
        if (distanceVec.norm()) {
            repulsiveForce = repulsiveForce + distanceVec * (1/distanceVec.norm());
        }
    }

    return Steering(Orientation3d(0), repulsiveForce);
}

std::map<std::string, MobilityData> Behavior_Separation::findNearByUAVs()
{
    auto mobilityDataReport = getAgent()->getLocalVehicleManager()->getMobilityData();
    std::map<std::string, Vehicle*> uavs = VehicleManager::getInstance()->getVehicles("UAV");

    std::map<std::string, MobilityData> nearby;

    for (auto it : mobilityDataReport) {
        MobilityData data = it.second;
        if ( (data.position - getAgent()->getPosition()).norm() <= m_searchRadius ){
            nearby.insert((nearby.end(), it));
        }
    }

    return nearby;
}

} // namespace LIMoSim
