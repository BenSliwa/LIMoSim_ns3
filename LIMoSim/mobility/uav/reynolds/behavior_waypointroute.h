#ifndef BEHAVIOR_WAYPOINTROUTE_H
#define BEHAVIOR_WAYPOINTROUTE_H

#include "LIMoSim/mobility/uav/reynolds/behavior.h"

namespace LIMoSim {

class UAV;

using Waypoints = std::vector<Vector3d>;

class Behavior_WaypointRoute : public Behavior
{
public:
    Behavior_WaypointRoute(Waypoints _waypoints,
                           double _arrivalRadius = 50.0,
                           double _checkpointRadius = 10,
                           std::string _agentId = "");

    // Behavior interface
public:
    Steering apply();

protected:
    Waypoints m_waypoints;
    double m_arrivalRadius;
    double m_checkpointRadius;
    std::size_t m_currentWaypointIdx;
};

} // namespace LIMoSim

#endif // BEHAVIOR_WAYPOINTROUTE_H
