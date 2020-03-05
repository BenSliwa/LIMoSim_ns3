#include "behavior_flee.h"
#include "LIMoSim/mobility/uav/uav.h"

namespace LIMoSim {

Behavior_Flee::Behavior_Flee(Target _target, std::string _agentId):
    Behavior("Flee", _agentId),
    m_target(_target)
{

}

Steering Behavior_Flee::apply()
{
    Vector3d currentVelocity = getAgent()->getVelocity();
    Orientation3d currentOrientation = getAgent()->getOrientation();
    Orientation3d currentOrientationVelocity = getAgent()->getOrientationVelocity();
    double velocityMax = getAgent()->getModel()->getLocomotion()->getVelocityMax();

    Vector3d desiredVelocity = (m_target.getPosition() - getAgent()->getPosition());
    if (desiredVelocity.norm() == 0) {
        desiredVelocity = Vector3d((double)rand(), (double)rand(), (double)rand()).normed() *
                velocityMax;
    } else {
        desiredVelocity = desiredVelocity.normed()* -1 * velocityMax;
    }
    Vector3d positionSteering = desiredVelocity - currentVelocity;

    Orientation3d desiredOrientationVelocity = computeDesiredOrientationVelocity(desiredVelocity, currentOrientation);
    Orientation3d orientationSteering = computeOrientationArrivalSteering(desiredOrientationVelocity, currentOrientationVelocity);


    return Steering(orientationSteering, positionSteering);
}

} // namespace LIMoSim
