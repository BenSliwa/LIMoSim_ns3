#include <cmath>
#include <algorithm>
#include "behavior_wander.h"
#include "behavior_collisionavoidance.h"
#include "LIMoSim/mobility/uav/uav.h"

namespace LIMoSim {

Behavior_Wander::Behavior_Wander(double _wanderingSpeed_mps, std::string _agentId):
    Behavior("Wander", _agentId),
    m_wanderingSpeed_mps(_wanderingSpeed_mps)
{

}

Steering Behavior_Wander::apply()
{
    Vector3d currentVelocity = getAgent()->getVelocity();
    Orientation3d currentOrientation = getAgent()->getOrientation();
    Orientation3d currentOrientationVelocity = getAgent()->getOrientationVelocity();

    Vector3d target = randomWanderSpherePoint();
    Vector3d desiredVelocity = (target - getAgent()->getPosition());
    if (desiredVelocity.norm() > 0) {
        desiredVelocity = desiredVelocity.normed() * m_wanderingSpeed_mps;
    }
    desiredVelocity = getAgent()->getModel()->getLocomotion()->constrainVelocity(desiredVelocity);
    Vector3d positionSteering = desiredVelocity - currentVelocity;

    Orientation3d desiredOrientationVelocity = computeDesiredOrientationVelocity(desiredVelocity, currentOrientation);
    Orientation3d orientationSteering = computeOrientationArrivalSteering(desiredOrientationVelocity, currentOrientationVelocity);

    return Steering(orientationSteering, positionSteering);
}

Vector3d Behavior_Wander::wanderSphereCenter()
{
    Orientation3d orientation = getAgent()->getOrientation();
    return getAgent()->getPosition() + Vector3d::fromSphere(orientation.pitch(), orientation.yaw(), m_wanderSphereDistance_m);
}

Vector3d Behavior_Wander::randomWanderSpherePoint()
{
    Vector3d sphereCenter = wanderSphereCenter();
    double randomPitch = 180 * (double)rand() / RAND_MAX;
    double randomYaw = 360 * (double)rand()/ RAND_MAX;
    Vector3d randomDirection = Vector3d::fromSphere(
                randomPitch,
                randomYaw,
                m_wanderSphereRadius_m
                );
    return sphereCenter + randomDirection;
}

} // namespace LIMoSim

