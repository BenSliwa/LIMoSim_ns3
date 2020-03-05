#include "mobilitymodel.h"
#include "vehicle.h"
#include "LIMoSim/world/vehiclemanager.h"

namespace LIMoSim {

MobilityModel::MobilityModel(std::string _agentId):
    m_agentId(_agentId)
{

}

Vehicle *MobilityModel::getAgent()
{
    return VehicleManager::getInstance()->getVehicle(m_agentId);
}

} // namespace LIMoSim
