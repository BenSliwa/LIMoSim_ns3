#include "behavior.h"
#include <algorithm>
#include <numeric>
#include "LIMoSim/mobility/uav/uav.h"
#include "LIMoSim/world/worldutils.h"
#include "LIMoSim/world/vehiclemanager.h"

namespace LIMoSim
{

Behavior::Behavior(std::string _name, std::string _agentId):
    m_agentId(_agentId),
    m_name(_name)
{
}

Behavior::~Behavior()
{

}

UAV *Behavior::getAgent()
{
    return dynamic_cast<UAV*>(VehicleManager::getInstance()->getVehicle(m_agentId));
}

std::string Behavior::getName()
{
    return m_name;
}

void Behavior::setAgent(std::string _agentId)
{
    m_agentId = _agentId;
}

std::string Behavior::setName(std::string _name)
{
    m_name = _name;
}

Steering Behavior::applyBehaviors(Behaviors _behaviors, bool _cleanUpBehaviors)
{
    Steerings steerings;
    std::transform(
                _behaviors.begin(),
                _behaviors.end(),
                std::back_inserter(steerings),
                [](Behavior* s)->Steering { return s->apply(); }
                );

    if (_cleanUpBehaviors){
        for (auto b : _behaviors) {
            delete b;
        }
    }

    Steering steering = std::accumulate(
                steerings.begin(),
                steerings.end(),
                Steering()
                );
    return steering;
}

Steering Behavior::applyNormalizedBehaviors(Behaviors _behaviors, bool _cleanUpBehaviors)
{
    Steerings steerings;
    std::transform(
                _behaviors.begin(),
                _behaviors.end(),
                std::back_inserter(steerings),
                [this](Behavior* s)->Steering { return getAgent()->getModel()->getLocomotion()->normalizeSteering(s->apply()); }
                );

    if (_cleanUpBehaviors){
        for (auto b : _behaviors) {
            delete b;
        }
    }

    Steering steering = std::accumulate(
                steerings.begin(),
                steerings.end(),
                Steering()
                );
    return steering;

}

Steering Behavior::applyNormalizedWeightedBehaviors(Behaviors _behaviors, Weights _weights, bool _cleanUpBehaviors)
{
    Steerings steerings;
    int pos = 0;
    std::transform(
                _behaviors.begin(),
                _behaviors.end(),
                std::back_inserter(steerings),
                [this, &pos, &_weights](Behavior* s)->Steering { return getAgent()->getModel()->getLocomotion()->normalizeSteering(s->apply()) * _weights[pos++]; }
                );

    if (_cleanUpBehaviors){
        for (auto b : _behaviors) {
            delete b;
        }
    }

    Steering steering = std::accumulate(
                steerings.begin(),
                steerings.end(),
                Steering()
                );
    return steering;
}

Orientation3d Behavior::computeDesiredOrientationVelocity(Vector3d _desiredVelocity, Orientation3d _currentOrientation)
{
    if (_desiredVelocity.norm() < 1e-4) {
        return Orientation3d(0);
    }
    Orientation3d desiredOrientation = Orientation3d::fromDirection(_desiredVelocity);

    // choose suitable yaw angle for shortest turn
    double normalizedYaw = normalizeAngle(_currentOrientation.yaw());
    _currentOrientation.setYaw(normalizedYaw);
    if (abs(normalizedYaw - desiredOrientation.yaw()) > abs(normalizedYaw - (desiredOrientation.yaw() + 360))) {
        desiredOrientation.setYaw(desiredOrientation.yaw() + 360);
    }
    return desiredOrientation - _currentOrientation;
}

Orientation3d Behavior::computeOrientationArrivalSteering(Orientation3d _desiredOrientationVelocity, Orientation3d _currentOrientationVelocity)
{
    _desiredOrientationVelocity.setPitch(sgn(_desiredOrientationVelocity.pitch()) * controlledDescent(0,90,0,getAgent()->getModel()->getLocomotion()->getOrientationVelocityMax().pitch(), abs(_desiredOrientationVelocity.pitch())));
    _desiredOrientationVelocity.setRoll(sgn(_desiredOrientationVelocity.roll()) * controlledDescent(0,90,0,getAgent()->getModel()->getLocomotion()->getOrientationVelocityMax().roll(), abs(_desiredOrientationVelocity.roll())));
    _desiredOrientationVelocity.setYaw(sgn(_desiredOrientationVelocity.yaw()) * controlledDescent(0,90,0,getAgent()->getModel()->getLocomotion()->getOrientationVelocityMax().yaw(), abs(_desiredOrientationVelocity.yaw()), true));
    return _desiredOrientationVelocity - _currentOrientationVelocity;
}

}
