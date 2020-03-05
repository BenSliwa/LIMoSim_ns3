#include "behavior_seekwithspeed.h"
#include "LIMoSim/mobility/uav/uav.h"


namespace LIMoSim {

Behavior_SeekWithSpeed::Behavior_SeekWithSpeed(Target _target, double _seekingSpeed, std::string _agentId):
    Behavior("Seek with speed", _agentId),
    m_target(_target),
    m_seekingSpeed(_seekingSpeed)
{

}

Steering Behavior_SeekWithSpeed::apply()
{
    Vector3d currentVelocity = getAgent()->getVelocity();
    Orientation3d currentOrientation = getAgent()->getOrientation();
    Orientation3d currentOrientationVelocity = getAgent()->getOrientationVelocity();

    Vector3d desiredVelocity = (m_target.getPosition() - getAgent()->getPosition());
    desiredVelocity = getAgent()->getModel()->getLocomotion()->modulateVelocity(desiredVelocity, m_seekingSpeed);
    Vector3d positionSteering = desiredVelocity - currentVelocity;

    Orientation3d desiredOrientationVelocity = computeDesiredOrientationVelocity(desiredVelocity, currentOrientation);
    Orientation3d orientationSteering = computeOrientationArrivalSteering(desiredOrientationVelocity, currentOrientationVelocity);

    return Steering(orientationSteering, positionSteering);
}

} // namespace LIMoSim
