#ifndef BEHAVIOR_RANDOMWALK_H
#define BEHAVIOR_RANDOMWALK_H

#include "LIMoSim/mobility/uav/reynolds/behavior.h"
#include "LIMoSim/mobility/uav/reynolds/target.h"
#include "LIMoSim/simulation/eventhandler.h"

namespace LIMoSim {
/**
 * @brief The Behavior_RandomWalk class
 * Travel along a randomly choosed direction with a randomly choosen speed
 * for a fixed amount of time.
 */
class Behavior_RandomWalk : public Behavior, EventHandler
{
public:
    Behavior_RandomWalk(std::string _agentId = "");
    Behavior_RandomWalk(double _xLength, double _yLength, std::string _agentId="");
    Behavior_RandomWalk(const Vector3d _min, const Vector3d _max, std::string _agentId="");

    // Behavior interface
    Steering apply();

    //
    void initialize();
    void handleEvent(Event* _event);
    void handleUpdateEvent(Event* _event);

protected:
    Vector3d m_travelDirection;
    double m_travelSpeed;
    double m_travelTime_s;
    double m_xMin = -100;
    double m_xMax = 300;
    double m_yMin = -100;
    double m_yMax = 300;
    double m_zMin = 0;
    double m_zMax = 200;
    double m_borderThreshold = 20;

    Event *m_updateTimer;
    double m_lastUpdate_s;

    bool m_initialized;

    void directionControl();
    void setNextDirection();
    void setNextSpeed();
};

} // namespace LIMoSim

#endif // BEHAVIOR_RANDOMWALK_H
