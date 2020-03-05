#include "behavior_randomwalk.h"
#include "LIMoSim/mobility/uav/reynolds/behavior_seekwithspeed.h"
#include "LIMoSim/mobility/uav/uav.h"

namespace LIMoSim {

Behavior_RandomWalk::Behavior_RandomWalk(std::string _agentId):
    Behavior("Random Walk", _agentId),
    m_travelTime_s(20),
    m_initialized(false)
{

}

Behavior_RandomWalk::Behavior_RandomWalk(double _xLength, double _yLength, std::string _agentId):
    Behavior("Random Walk", _agentId),
    m_travelTime_s(20),
    m_initialized(false)
{
    m_xMin = -_xLength / 2;
    m_xMax = _xLength / 2;
    m_yMin = -_yLength / 2;
    m_yMax = _yLength / 2;
    m_zMin = 25.0;
    m_zMax = 35.0;
}

Behavior_RandomWalk::Behavior_RandomWalk(const Vector3d _min,
                                         const Vector3d _max,
                                         std::string _agentId):
    Behavior("Random Walk", _agentId),
    m_travelTime_s(20),
    m_initialized(false)
{
    m_xMin = _min.x;
    m_xMax = _max.x;
    m_yMin = _min.y;
    m_yMax = _max.y;
    m_zMin = _min.z;
    m_zMax = _max.z;
}

Steering Behavior_RandomWalk::apply()
{
    if (!m_initialized) {
        initialize();
    }

    directionControl();

    Steering s = Behavior_SeekWithSpeed(getAgent()->getPosition() + m_travelDirection * m_travelSpeed, m_travelSpeed, m_agentId).apply();
    return s;
}

void Behavior_RandomWalk::initialize()
{
    setNextSpeed();
    setNextDirection();
    m_updateTimer = new Event(m_travelTime_s, this, "update");
    scheduleEvent(m_updateTimer);
    m_initialized = true;
}

void Behavior_RandomWalk::handleEvent(Event *_event)
{
    if (_event->getInfo() == m_updateTimer->getInfo()) {
        handleUpdateEvent(_event);
    } else {
        delete _event;
    }
}

void Behavior_RandomWalk::handleUpdateEvent(Event *_event)
{
    setNextSpeed();
    setNextDirection();
    m_lastUpdate_s = _event->getTimestamp();
    scheduleEvent(_event, m_travelTime_s);
}

void Behavior_RandomWalk::directionControl()
{
    Vector3d position = getAgent()->getPosition();

    if(
             ((position.x < m_xMin + m_borderThreshold) && (position.x - m_xMin - m_borderThreshold < 0) && (m_travelDirection.x < 0)) ||
             ((position.x > m_xMax - m_borderThreshold) && (position.x - m_xMax - m_borderThreshold > 0) && (m_travelDirection.x > 0))
            ) {
        m_travelDirection.x *= -1;
    }

    if(
            ((position.y < m_yMin + m_borderThreshold) && (position.y - m_yMin - m_borderThreshold < 0) && (m_travelDirection.y < 0)) ||
            ((position.y > m_yMax - m_borderThreshold) && (position.y - m_yMax - m_borderThreshold > 0) && (m_travelDirection.y > 0))
           ) {
        m_travelDirection.y *= -1;
    }

    if(
            ((position.z < m_zMin + m_borderThreshold) && (position.z - m_zMin - m_borderThreshold < 0) && (m_travelDirection.z < 0)) ||
            ((position.z > m_zMax - m_borderThreshold) && (position.z - m_zMax - m_borderThreshold > 0) && (m_travelDirection.z > 0))
           ) {
        m_travelDirection.z *= -1;
    }

}

void Behavior_RandomWalk::setNextDirection()
{
    double randomPitch = 180 * (double)rand() / RAND_MAX;
    double randomYaw = 360 * (double)rand() / RAND_MAX;
    m_travelDirection = Vector3d::fromSphere(
                randomPitch, //90,
                randomYaw,
                1
                );
}

void Behavior_RandomWalk::setNextSpeed()
{
    m_travelSpeed = getAgent()->getModel()->getLocomotion()->getVelocityMax() * (double)rand() / RAND_MAX;
}

} // namespace LIMoSim
