#include "behavior_arrive.h"
#include "behavior_collisionavoidance.h"
#include "LIMoSim/mobility/uav/uav.h"
#include "LIMoSim/world/worldutils.h"

namespace LIMoSim {

Behavior_Arrive::Behavior_Arrive(Target _target, double _arrivalRadius, std::string _agentId):
    Behavior("Arrive", _agentId),
    m_target(_target),
    m_arrivalRadius(_arrivalRadius)
{

}

Behavior_Arrive::Behavior_Arrive(Target _target, std::string _agentId):
    Behavior("Arrive", _agentId),
    m_target(_target),
    m_arrivalRadius(60.0)
{

}

Steering Behavior_Arrive::apply()
{
    double maxSpeed = getAgent()->getModel()->getLocomotion()->getVelocityMax();

    Vector3d desiredVelocity = (m_target.getPosition() - getAgent()->getPosition());
    if (desiredVelocity.norm() < 1e-1) {
        return Steering();
        return Steering((getAgent()->getOrientationVelocity() * -1) - getAgent()->getOrientationVelocity(),
                        (getAgent()->getVelocity() * -1) -getAgent()->getVelocity());
    }
    desiredVelocity = controlledDescent(0, m_arrivalRadius,0, maxSpeed,desiredVelocity);
    Vector3d positionSteering = desiredVelocity - getAgent()->getVelocity();

    return Steering(Orientation3d(0), positionSteering);
}

} // namespace LIMoSim
