#include "behavior_waypointroute.h"
#include "LIMoSim/mobility/uav/uav.h"
#include "behavior_arrive.h"

namespace LIMoSim {

Behavior_WaypointRoute::Behavior_WaypointRoute(Waypoints _waypoints,
                                               double _arrivalRadius,
                                               double _checkpointRadius,
                                               std::string _agentId):
    Behavior("WaypointRoute", _agentId),
    m_waypoints(_waypoints),
    m_arrivalRadius(_arrivalRadius),
    m_checkpointRadius(_checkpointRadius),
    m_currentWaypointIdx(0)
{

}

Steering Behavior_WaypointRoute::apply()
{
    if (m_currentWaypointIdx < m_waypoints.size() -1 &&
            (m_waypoints.at(m_currentWaypointIdx) - getAgent()->getPosition()).norm() < m_checkpointRadius) {
        m_currentWaypointIdx++;
    }

    return Behavior_Arrive(m_waypoints.at(m_currentWaypointIdx),m_arrivalRadius,getAgent()->getId()).apply();
}

} // namespace LIMoSim
