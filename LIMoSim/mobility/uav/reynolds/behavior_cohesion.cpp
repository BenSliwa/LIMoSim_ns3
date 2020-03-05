#include "behavior_cohesion.h"
#include "behaviors.h"

#include "LIMoSim/mobility/uav/uav.h"

namespace LIMoSim {

Behavior_Cohesion::Behavior_Cohesion(double _searchRadius, std::string _agentId):
    Behavior("Cohesion", _agentId),
    m_searchRadius(_searchRadius)
{

}

Behavior_Cohesion::Behavior_Cohesion(std::string _name, double _searchRadius, std::string _agentId):
    Behavior(_name, _agentId),
    m_searchRadius(_searchRadius)
{

}

Steering Behavior_Cohesion::apply()
{
    MobilityDataMap nearbyUAVs = findNearByUAVs();
    Vector3d meanPosition;

    if(!nearbyUAVs.size()) {
        return Steering(Orientation3d(), (getAgent()->getVelocity() * -1) -getAgent()->getVelocity());
    }
    // compute mean position
    for (auto it = nearbyUAVs.begin(); it != nearbyUAVs.end(); it++) {
        MobilityData data = it->second;
        meanPosition = meanPosition + data.position;
    }

    meanPosition = meanPosition / (double)nearbyUAVs.size();

    //TODO: avoid behavior creation and destruction on each call
    Behavior* arrive = new Behavior_Arrive(meanPosition, m_agentId);
    Steering steering = makeSelfAligned(arrive,m_agentId)->apply();
    delete arrive;
    return steering;
}

std::map<std::string, MobilityData> Behavior_Cohesion::findNearByUAVs()
{
    auto mobilityDataReport = getAgent()->getLocalVehicleManager()->getMobilityData();

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
