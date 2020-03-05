#ifndef BEHAVIOR_RANDOMWAYPOINT_H
#define BEHAVIOR_RANDOMWAYPOINT_H

#include <random>

#include "LIMoSim/mobility/uav/reynolds/behavior.h"
#include "LIMoSim/simulation/eventhandler.h"

namespace LIMoSim {

typedef std::uniform_int_distribution<int> RandomIntDist;

class Behavior_RandomWaypoint : public Behavior
{
public:
    Behavior_RandomWaypoint(std::string _agentId = "");
    Behavior_RandomWaypoint(const Vector3d _min,
                            const Vector3d _max,
                            std::string _agentId = "");

    // Behavior interface
    Steering apply();

    Vector3d getWaypoint();

protected:
    Vector3d m_waypoint;
    double m_travelSpeed;
    double m_maxSpeed_mps = 20;
    double m_xMin = 0;
    double m_xMax = 300;
    double m_yMin = 0;
    double m_yMax = 300;
    double m_zMin = 5;
    double m_zMax = 200;
    double m_borderThreshold = 20;
    double m_arrivalThreshold = 5;
    double m_arrivalRadius = 20;

    std::random_device m_rdev;
    std::mt19937 m_rgen;
    RandomIntDist m_distX;
    RandomIntDist m_distY;
    RandomIntDist m_distZ;
    RandomIntDist m_distSpeed;
    RandomIntDist m_distArrive;

    Vector3d selectRandomwaypoint();
};

} // namespace LIMoSim

#endif // BEHAVIOR_RANDOMWAYPOINT_H
