#include "behavior_cohesion2dregionwise.h"
#include "LIMoSim/mobility/uav/uav.h"

namespace LIMoSim {

Behavior_Cohesion2DRegionwise::Behavior_Cohesion2DRegionwise(Vector3d _regionTopLeft,
                                                             double _regionHeight,
                                                             double _regionWidth,
                                                             std::string _agentId):
    Behavior_Cohesion2D("Cohesion2DRegionwise", 0, _agentId),
    m_regionTopLeft(_regionTopLeft),
    m_regionHeight(_regionHeight),
    m_regionWidth(_regionWidth)
{

}

std::map<std::string, MobilityData> Behavior_Cohesion2DRegionwise::findNearByUAVs()
{
    auto mobilityDataReport = getAgent()
            ->getLocalVehicleManager()
            ->getMobilityData();

    std::map<std::string, MobilityData> nearby;

    for (auto it : mobilityDataReport) {
        MobilityData data = it.second;
        if (isInBounds(data.position)) {
            nearby.insert(nearby.end(), it);
        }
    }

    return nearby;

}

bool Behavior_Cohesion2DRegionwise::isInBounds(Vector3d _position)
{
    return (m_regionTopLeft.x <= _position.x && _position.x <= m_regionTopLeft.x + m_regionWidth) &&
            (m_regionTopLeft.y - m_regionHeight <= _position.y && _position.y <= m_regionTopLeft.y);
}

} // namespace LIMoSim
