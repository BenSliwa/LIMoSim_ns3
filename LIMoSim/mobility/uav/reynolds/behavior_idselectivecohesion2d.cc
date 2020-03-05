#include "behavior_idselectivecohesion2d.h"

#include <algorithm>

#include "LIMoSim/mobility/uav/uav.h"

namespace LIMoSim {

Behavior_IdSelectiveCohesion2D::Behavior_IdSelectiveCohesion2D(std::vector<std::string> _relevantIds,
                                                               double _searchRadius,
                                                               std::string _agentId):
    Behavior_Cohesion2D ("IdSelectiveCohesion2D", _searchRadius, _agentId),
    m_relevantIds(_relevantIds)
{

}

std::map<std::string, MobilityData> Behavior_IdSelectiveCohesion2D::findNearByUAVs()
{
    auto mobilityDataReport = getAgent()->getLocalVehicleManager()->getMobilityData();

    std::map<std::string, MobilityData> nearby;

    for (auto it : mobilityDataReport) {
        MobilityData data = it.second;
        std::find(m_relevantIds.begin(), m_relevantIds.end(), data.name);
        if ( (data.position - getAgent()->getPosition()).norm() <= m_searchRadius &&
             std::find(m_relevantIds.begin(), m_relevantIds.end(), data.name) != m_relevantIds.end()){
            nearby.insert(nearby.end(), it);
        }
    }

    return nearby;
}

} // namespace LIMoSim
