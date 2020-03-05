#include "behavior_selfalignment.h"
#include "LIMoSim/mobility/uav/uav.h"
#include "LIMoSim/mobility/uav/reynolds/behavior_composition.h"

namespace LIMoSim {

Behavior_SelfAlignment::Behavior_SelfAlignment(std::string _agentId):
    Behavior("Self Alignemnt", _agentId)
{

}

Behavior *Behavior_SelfAlignment::composeWith(Behavior *_behavior, std::string _agentId)
{
    if (!_agentId.empty() && !_behavior->getAgent()) {
        _behavior->setAgent(_agentId);
    }
    if (_agentId.empty() && _behavior->getAgent()) {
        _agentId = _behavior->getAgent()->getId();
    }
    return new Behavior_Composition(
                _behavior->getName() + " with Self Alignment",
                Behaviors {
                    new Behavior_SelfAlignment(_agentId),
                    _behavior
                },
                CompositionMode::SUPERPOSITION,
                _agentId
                );
}

Steering Behavior_SelfAlignment::apply()
{
    Vector3d currentVelocity = getAgent()->getVelocity();
    Orientation3d currentOrientationVelocity = getAgent()->getOrientationVelocity();

    if (currentVelocity.norm() < 1e-2){
        // stop orientation movements
        return Steering(Orientation3d(
                            currentOrientationVelocity.pitch() * -2,
                            currentOrientationVelocity.roll() * -2,
                            currentOrientationVelocity.yaw() * -2
                            ),
                        Vector3d()
                        );
    }

    Orientation3d currentOrientation = getAgent()->getOrientation();
    Orientation3d desiredOrientationVelocity = computeDesiredOrientationVelocity(currentVelocity, currentOrientation);
    Orientation3d orientationSteering = computeOrientationArrivalSteering(desiredOrientationVelocity, currentOrientationVelocity);

    return Steering(orientationSteering, Vector3d());
}

Orientation3d Behavior_SelfAlignment::alignOnVelocity(Vector3d _velocity)
{
    Orientation3d currentOrientation = getAgent()->getOrientation();
    Orientation3d currentOrientationVelocity = getAgent()->getOrientationVelocity();
    Orientation3d desiredOrientationVelocity = computeDesiredOrientationVelocity(_velocity, currentOrientation);
    Orientation3d orientationSteering = computeOrientationArrivalSteering(desiredOrientationVelocity, currentOrientationVelocity);

    return orientationSteering;
}

} // namespace LIMoSim
