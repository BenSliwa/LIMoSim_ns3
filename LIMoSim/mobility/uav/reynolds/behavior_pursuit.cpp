#include "behavior_pursuit.h"
#include "behaviors.h"
#include "LIMoSim/mobility/uav/uav.h"

#include "LIMoSim/world/vehiclemanager.h"

namespace LIMoSim {

Behavior_Pursuit::Behavior_Pursuit(Target _target, std::string _agentId):
    Behavior("Pursuit", _agentId),
    m_target(_target)
{

}

Steering Behavior_Pursuit::apply()
{
    // TODO: avoid instantiating a behavior on each call
    Behavior * seek = new Behavior_Seek(predictTargetPosition(), m_agentId);
    Steering steering = makeSelfAligned(seek)->apply();
    delete seek;
    return steering;
}

Vector3d Behavior_Pursuit::predictTargetPosition()
{
    static double turningParameter = 0.1;

    double distance = (m_target.getPosition() - getAgent()->getPosition()).norm();
    double predictionInterval = distance * turningParameter;

    return m_target.getPosition() + m_target.getVelocity() * predictionInterval;
}


} // namespace LIMoSim
