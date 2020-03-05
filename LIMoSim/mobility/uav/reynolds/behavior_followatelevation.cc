#include "behavior_followatelevation.h"

#include "behaviors.h"

#include "LIMoSim/mobility/uav/uav.h"
#include "LIMoSim/mobility/car/strategic/truckdelivery.h"

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

Vector3d Behavior_FollowAtElevation::predictPosition(double _future)
{
    Vehicle * vehicle = VehicleManager::getInstance()->getVehicle(m_targetAgentId);

    if (vehicle->getType() == "Car") {

        auto model = dynamic_cast<Car*>(vehicle)->getStrategicModel();
        auto truckdl = dynamic_cast<TruckDelivery*>(model);
        if (truckdl) {

            std::vector<Vector3d> previousPositions;
            previousPositions.push_back(vehicle->getPosition() + vehicle->getAcceleration() * 1);
            Vector3d waypoint = truckdl->getcurrentWaypoint();

            for (int step = 1; step < _future; step++) {

                Vector3d prevPos = previousPositions.back();

                if ((prevPos - waypoint).norm() > 30) {
                    // waypoint prediction
                    Vector3d stepPos = prevPos +
                            ((waypoint - prevPos).norm() ? ((waypoint - prevPos)/(waypoint - prevPos).norm())  : Vector3d())
                            * vehicle->getVelocity().norm() * 1;
                    previousPositions.push_back(stepPos);
                } else  {
                    // extrapolation
                    Vector3d v;
                    std::size_t numPrevSteps = previousPositions.size();
                    for (std::size_t i = 1; i < numPrevSteps; i++) {
                        v = v + ((previousPositions.at(i) - previousPositions.at(i-1)).norm() ?
                                    (previousPositions.at(i) - previousPositions.at(i-1))/(previousPositions.at(i) - previousPositions.at(i-1)).norm():
                                    Vector3d());
                    }
                    v = v/numPrevSteps + prevPos;
                    previousPositions.push_back(v);
                }
            }

            return previousPositions.back();


    //            Vector3d currPos = m_vehiclePreviousPositions.at(_vehicleId).front();
    //            return currPos + ((waypoint - currPos)/(waypoint - currPos).norm()) * vehicle->getVelocity().norm() * _future;
        } else {
            return Vector3d();
        }
    }
    return Vector3d();
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
            double t = (getAgent()->getPosition() - VehicleManager::getInstance()->getVehicle(m_targetAgentId)->getPosition()).norm() / std::max(getAgent()->getVelocity().norm(),10.0); // getAgent()->getModel()->getLocomotion()->getVelocityMax();
//            std::cout << "predictin horizon: " << t << std::endl;
//            double t = 9;
//            predictedTargetPosition = predictPosition(t+1);
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

