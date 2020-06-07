#include "behavior_followatelevation.h"

#include "behaviors.h"

#include "LIMoSim/mobility/uav/uav.h"

#include "LIMoSim/world/vehiclemanager.h"
#include "LIMoSim/world/worldutils.h"

namespace LIMoSim {

Behavior_FollowAtElevation::Behavior_FollowAtElevation(std::string _targetAgentId,
        double _elevation,
        double _followRadius,
        bool _predictionEnabled,
        std::string _agentId
        ):
    Behavior("FollowFromAbove", _agentId),
    m_targetAgentId(_targetAgentId),
    m_elevation_m(_elevation),
    m_followRadius_m(_followRadius),
    m_predictionEnabled(_predictionEnabled),
    m_selfAlignment(new Behavior_SelfAlignment(_agentId))
{

}

Behavior_FollowAtElevation::~Behavior_FollowAtElevation()
{
    delete m_selfAlignment;
}

Vector3d Behavior_FollowAtElevation::predictTargetPosition()
{
    static double turningParameter = 0.01;
    auto mobilityData = getAgent()->getLocalVehicleManager()->getMobilityData();

    if (mobilityData.count(m_targetAgentId)) {
        double distance = (mobilityData.at(m_targetAgentId).position - getAgent()->getPosition()).norm();
        double predictionInterval = distance * turningParameter;
        return mobilityData.at(m_targetAgentId).position + mobilityData.at(m_targetAgentId).velocity * predictionInterval;
    } else {
        // predict own position
        double predictionInterval = getAgent()->getVelocity().norm() * turningParameter;
        return getAgent()->getPosition() + getAgent()->getVelocity() * predictionInterval;
    }
}

} // namespace LIMoSim


// Behavior interface
namespace LIMoSim {
Steering Behavior_FollowAtElevation::apply()
{
    double maxSpeed = getAgent()->getModel()->getLocomotion()->getVelocityMax();
    auto mobilityData = getAgent()->getLocalVehicleManager()->getMobilityData();

    if (mobilityData.count(m_targetAgentId)) {
        // If mobility data of target agent is known, proceed to following
        Vector3d predictedTargetPosition;
        if (m_predictionEnabled) {
            predictedTargetPosition = predictTargetPosition();
        } else {
            predictedTargetPosition = Vector3d (mobilityData.at(m_targetAgentId).position.x,
                                                                 mobilityData.at(m_targetAgentId).position.y,
                                                                 m_elevation_m);
        }

        Vector3d targetedPosition = Vector3d (predictedTargetPosition.x,
                                              predictedTargetPosition.y,
                                              m_elevation_m);

        Vector3d desiredVelocity = (targetedPosition - getAgent()->getPosition());
        if (desiredVelocity.norm() < 1e-4) {
            return Steering();
        }
        desiredVelocity = controlledDescent(0, m_followRadius_m,0, maxSpeed,desiredVelocity);
        Vector3d positionSteering = desiredVelocity - getAgent()->getVelocity();

        return  Steering(Orientation3d(0), positionSteering) + m_selfAlignment->apply();
    } else {
        // If not then stop
        return Steering(Orientation3d(), (getAgent()->getVelocity() * -1) -getAgent()->getVelocity());
    }
}


void Behavior_FollowAtElevation::setAgent(std::string _agentId)
{
    Behavior::setAgent(_agentId);
    m_selfAlignment->setAgent(_agentId);
}
}

