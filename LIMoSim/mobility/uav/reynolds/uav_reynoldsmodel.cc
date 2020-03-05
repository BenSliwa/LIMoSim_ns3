#include "uav_reynoldsmodel.h"
#include "behavior_circlearound.h"
#include "behavior_randomwaypoint.h"
#include "LIMoSim/mobility/uav/uav.h"
#include "LIMoSim/world/vehiclemanager.h"

namespace LIMoSim {

UAV_ReynoldsModel::UAV_ReynoldsModel(std::string _agentId):
    ReynoldsModel(_agentId, nullptr, new UAV_Locomotion(_agentId)),
    m_carried(false),
    m_carryingAgent(nullptr)
{}

UAV_ReynoldsModel::~UAV_ReynoldsModel()
{

}

Vector3d UAV_ReynoldsModel::getWaypoint()
{
    if (getBehavior()->getName() == "RandomWaypoint") {
        return dynamic_cast<Behavior_RandomWaypoint*>(getBehavior())->getWaypoint();
    }
}

Vehicle *UAV_ReynoldsModel::getCarryingAgent()
{
    return m_carryingAgent;
}

bool UAV_ReynoldsModel::agentIsCarried()
{
    return m_carried;
}

void UAV_ReynoldsModel::setCarryingAgent(std::string _vehicleId)
{
    m_carried = true;
    Vehicle* vehicle = VehicleManager::getInstance()->getVehicle(_vehicleId);
    m_carryingAgent = vehicle;
}

void UAV_ReynoldsModel::unsetCarrying()
{
    m_carried = false;
    m_carryingAgent = nullptr;
}

UAV_Locomotion *UAV_ReynoldsModel::getLocomotion()
{
    return dynamic_cast<UAV_Locomotion*>(ReynoldsModel::getLocomotion());
}


}
