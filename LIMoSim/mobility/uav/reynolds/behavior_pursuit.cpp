#include "behavior_pursuit.h"
#include "behaviors.h"
#include "LIMoSim/mobility/uav/uav.h"

#include "LIMoSim/world/vehiclemanager.h"
#include "LIMoSim/mobility/car/strategic/strategicmodel.h"
#include "LIMoSim/mobility/car/strategic/truckdelivery.h"

namespace LIMoSim {

Behavior_Pursuit::Behavior_Pursuit(Target _target, std::string _agentId):
    Behavior("Pursuit", _agentId),
    m_target(_target)
{

}

Steering Behavior_Pursuit::apply()
{
    // TODO: avoid instantiating a behavior on each call
//    Behavior * seek = new Behavior_Seek(predictTargetPosition(), m_agentId);
    double t = (getAgent()->getPosition() - m_target.getPosition()).norm() / std::max(getAgent()->getVelocity().norm(),10.0); // getAgent()->getModel()->getLocomotion()->getVelocityMax();
    Behavior * seek = new Behavior_Seek(predictPosition(t), m_agentId);
    Steering steering = makeSelfAligned(seek)->apply();
    delete seek;
    return steering;
}

Vector3d Behavior_Pursuit::predictTargetPosition()
{
    static double turningParameter = 0.1;

    double distance = (m_target.getPosition() - getAgent()->getPosition()).norm();
    double predictionInterval = distance * turningParameter;

    return m_target.getPosition() + m_target.getVelocity() * predictionInterval;
}

Vector3d Behavior_Pursuit::predictPosition(double _future)
{
    Vehicle * vehicle = VehicleManager::getInstance()->getVehicle(m_target.getVehicleId());

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
}


} // namespace LIMoSim
