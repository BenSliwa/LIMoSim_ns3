#include "behavior_seek.h"
#include "behavior_collisionavoidance.h"
#include "LIMoSim/mobility/uav/uav.h"
#include "LIMoSim/world/worldutils.h"

namespace LIMoSim {

Behavior_Seek::Behavior_Seek(Target _target, std::string _agentId):
    Behavior("Seek", _agentId),
    m_target(_target)
{

}

Steering Behavior_Seek::apply()
{
    Vector3d desiredVelocity = (m_target.getPosition() - getAgent()->getPosition());
    desiredVelocity = getAgent()->getModel()->getLocomotion()->constrainVelocity(desiredVelocity);
    Vector3d positionSteering = desiredVelocity - getAgent()->getVelocity();
    return Steering(Orientation3d(0), positionSteering);
}

} // namespace LIMoSim
