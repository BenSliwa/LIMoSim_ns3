#include "behavior_delivery.h"

#include <algorithm>

#include "LIMoSim/mobility/uav/reynolds/behaviors.h"
#include "LIMoSim/mobility/uav/uav.h"
#include "LIMoSim/mobility/deliverytruck/deliverytruck.h"
#include "LIMoSim/world/world.h"
#include "LIMoSim/world/vehiclemanager.h"

#include "demo/deliverylistservice.h"

namespace LIMoSim {
namespace delivery {
namespace Behaviors {

Behavior_Delivery::Behavior_Delivery(std::string _truckId, bool _predictionEnabled, std::string _agentId):
    Behavior("Delivery", _agentId),
    m_truckId(_truckId),
    m_followingHeight (7.0),
    m_secureHeight(50.0),
    m_predictionEnabled(_predictionEnabled)
{
    m_truck = dynamic_cast<DeliveryTruck*>(VehicleManager::getInstance()->getVehicle(m_truckId));
}

std::string Behavior_Delivery::getDeliveryTargetId() const
{
    return m_deliveryTargetId;
}

double Behavior_Delivery::getDeliveryReach() const
{
    return m_uavRangeMax;
}

void Behavior_Delivery::startNextDelivery()
{
    m_deliveryTargetId = delivery::DeliveryListService::getInstance()->getNextDroneDeliveryTarget();
    std::cout << "next UAV delivery target: " << m_deliveryTargetId << std::endl;
    if (m_deliveryTargetId.empty()) {
        return;
    }
    World* p_world = World::getInstance();
    m_deliveryTarget = p_world->getBuildings().at(m_deliveryTargetId);
    const std::vector<Node*> nodes = m_deliveryTarget->getNodes();
    std::vector<Vector3d> coords;
    std::transform(
                nodes.begin(),
                nodes.end(),
                std::back_inserter(coords),
                [](Node* n) -> Vector3d {return n->getPosition();}
                );
    Vector3d sum;
    for (std::size_t i = 0; i < coords.size(); i++) {
        sum = sum + coords.at(i);
    }
    m_targetCoordinates = (sum / coords.size()) +
            Vector3d(0,0,m_deliveryTarget->getHeight() + 20);
}

void Behavior_Delivery::setDeliveryCompleteCallback(std::function<void (std::string)> _callback)
{
    m_deliveryCompleteCallback = _callback;
}


Steering Behavior_Delivery::apply()
{
    Vector3d agentPos = getAgent()->getPosition();
    // move towards suitable target and check status changes
    if (!m_deliveryTargetId.empty()) {
        if ((getAgent()->getPosition() - m_targetCoordinates).norm() < 10) {
            // delivery was just completed
            DeliveryListService::getInstance()->notifyDelivery(m_deliveryTargetId);
            m_deliveryCompleteCallback(m_deliveryTargetId);
            m_deliveryTargetId.clear();
            return Behavior_NoOp().apply();
        } else {
            // delivery is pending
            if ((getAgent()->getPosition() - m_targetCoordinates).norm() < m_uavRangeMax) {
                // run delivery

                if ( (Vector3d(agentPos.x, agentPos.y) -
                      Vector3d(m_targetCoordinates.x, m_targetCoordinates.y)).norm() > 20){
                    if ( std::abs(agentPos.z - m_secureHeight) > 10.0) {
                        // Ascend
                        return Behavior_Arrive(Vector3d(agentPos.x, agentPos.y, m_secureHeight),
                                               50,
                                               getAgent()->getId()).apply();
                    } else {
                        auto b = new Behavior_Arrive(m_targetCoordinates,
                                                     50,
                                                     getAgent()->getId());
                        auto s = makeSelfAligned(b)->apply();
                        delete b;
                        return s;
                    }
                }
                else {
                    // Descend
                    return Behavior_Arrive(m_targetCoordinates,50, getAgent()->getId()).apply();
                }
            } else {
                // keep following truck untill target is in delivery reach
                return Behavior_Separation(20,getAgent()->getId()).apply() + Behavior_FollowAtElevation(m_truckId,m_followingHeight,5,m_predictionEnabled,getAgent()->getId()).apply();
//                return Behavior_Pursuit(Target(m_truckId), getAgent()->getId()).apply();
            }
        }
    } else {
        if ((getAgent()->getPosition() - Target(m_truckId).getPosition()).norm() < 10) {
            // just reached the truck
//            startNextDelivery();
            return Behavior_NoOp().apply();
        } else {
            if (distance2D(agentPos, Target(m_truckId).getPosition()) > 20) {
                if (std::abs(agentPos.z - m_secureHeight) > 10.0) {
                    // Ascend
                    return Behavior_Arrive(Vector3d(agentPos.x, agentPos.y, m_secureHeight),
                                           50,
                                           getAgent()->getId()).apply();
                } else {
                    // Move towards truck
//                    return Behavior_FollowAtElevation(m_truckId,m_secureHeight,5,m_predictionEnabled,getAgent()->getId()).apply();
                    // Move towards truck's next delivery target
                    return Behavior_Arrive(m_truck->getCurrentDeliveryTargetPosition(),50, getAgent()->getId()).apply();
                }
            } else {
                // Descend to truck
                return Behavior_Separation(20,getAgent()->getId()).apply() +
                        Behavior_FollowAtElevation(m_truckId,m_followingHeight,5,m_predictionEnabled,getAgent()->getId()).apply();
        //        return Behavior_Pursuit(Target(m_truckId), getAgent()->getId()).apply();
            }
        }
    }
}

double Behavior_Delivery::distance2D(Vector3d a, Vector3d b)
{
    return (Vector3d(a.x, a.y) - Vector3d(b.x, b.y)).norm();
}

void Behavior_Delivery::clearDeliveryTargetId()
{
    m_deliveryTargetId.clear();
}

DeliveryTruck *Behavior_Delivery::getTruck()
{
    return m_truck;
}

bool Behavior_Delivery::getPredictionEnabled()
{
    return m_predictionEnabled;
}

std::string Behavior_Delivery::getDeliveryTargetId()
{
    return m_deliveryTargetId;
}

void Behavior_Delivery::setTruckId(std::string _id)
{
    m_truckId = _id;
    m_truck = dynamic_cast<DeliveryTruck*>(VehicleManager::getInstance()->getVehicle(m_truckId));
}

void Behavior_Delivery::setPredictionEnabled(bool _enabled)
{
    m_predictionEnabled = _enabled;
}

void Behavior_Delivery::setAgentId(std::string _agentId)
{
    setAgent(_agentId);
}

} // namespace Behaviors
} // namespace Demo
} // namespace LIMoSim

