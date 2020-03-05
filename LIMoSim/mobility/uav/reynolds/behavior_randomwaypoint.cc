#include "behavior_randomwaypoint.h"

#include "behavior_arrive.h"
#include "behavior_selfalignment.h"

#include "LIMoSim/mobility/uav/uav.h"

namespace LIMoSim {

Behavior_RandomWaypoint::Behavior_RandomWaypoint(std::string _agentId):
    Behavior("RandomWaypoint", _agentId),
    m_rgen(std::mt19937(m_rdev())),
    m_distArrive(50,200),
    m_distX(m_xMin, m_xMax),
    m_distY(m_yMin, m_yMax),
    m_distZ(m_zMin, m_zMax),
    m_distSpeed(1,m_maxSpeed_mps)
{
    m_waypoint = Vector3d(m_distX(m_rgen), m_distY(m_rgen), m_distZ(m_rgen));
}

Behavior_RandomWaypoint::Behavior_RandomWaypoint(const Vector3d _min,
                                                 const Vector3d _max,
                                                 std::string _agentId):
    Behavior("RandomWaypoint", _agentId),
    m_xMin(_min.x + m_borderThreshold),
    m_xMax(_max.x - m_borderThreshold),
    m_yMin(_min.y + m_borderThreshold),
    m_yMax(_max.y - m_borderThreshold),
    m_zMin(_min.z + m_borderThreshold),
    m_zMax(_max.z - m_borderThreshold),
    m_rgen(std::mt19937(m_rdev())),
    m_distArrive(50,200),
    m_distX(m_xMin, m_xMax),
    m_distY(m_yMin, m_yMax),
    m_distZ(m_zMin, m_zMax),
    m_distSpeed(5,m_maxSpeed_mps)
{
    m_waypoint = Vector3d(m_distX(m_rgen), m_distY(m_rgen), m_distZ(m_rgen));
}

Steering Behavior_RandomWaypoint::apply()
{
    if ((getAgent()->getPosition() - m_waypoint).norm() <= m_arrivalThreshold) {
        m_waypoint = Vector3d(m_distX(m_rgen), m_distY(m_rgen), m_distZ(m_rgen));
        m_waypoint.z = getAgent()->getPosition().z;
        m_travelSpeed = m_distSpeed(m_rgen);
        m_arrivalRadius = m_distArrive(m_rgen);
        getAgent()->getModel()->getLocomotion()->setVelocityMax(m_travelSpeed);
    }

    Behavior_Arrive arrive (m_waypoint,m_arrivalRadius,m_agentId);
    Behavior_SelfAlignment selfAlign(m_agentId);

    Steering s = applyBehaviors(Behaviors {&arrive, &selfAlign}, false);
//    s.position.z = 0;
    return s;
}

Vector3d Behavior_RandomWaypoint::getWaypoint()
{
    return m_waypoint;
}

Vector3d Behavior_RandomWaypoint::selectRandomwaypoint()
{
    return Vector3d(m_distX(m_rgen), m_distY(m_rgen), m_distZ(m_rgen));
}

} // namespace LIMoSim
