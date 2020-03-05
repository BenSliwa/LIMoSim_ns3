#include "behavior_evasion.h"
#include "behavior_flee.h"
#include "LIMoSim/mobility/uav/uav.h"

namespace LIMoSim {

Behavior_Evasion::Behavior_Evasion(Target _target, std::string _agentId):
    Behavior("Evasion", _agentId),
    m_target(_target)
{

}

Steering Behavior_Evasion::apply()
{
    return Behavior_Flee(predictTargetPosition(), m_agentId).apply();
}

Vector3d Behavior_Evasion::predictTargetPosition()
{
    static double turningParameter = 0.01;

    double distance = (m_target.getPosition() - getAgent()->getPosition()).norm();
    double predictionInterval = distance * turningParameter;

    return m_target.getPosition() + m_target.getVelocity() * predictionInterval;
}

} // namespace LIMoSim
