#include "behavior_dock.h"

#include "LIMoSim/world/vehiclemanager.h"
#include "LIMoSim/mobility/uav/reynolds/behavior_arrive.h"

namespace LIMoSim {
namespace delivery {
namespace Behaviors {

Behavior_Dock::Behavior_Dock(std::string _dockTargetId, std::string _agentId):
    Behavior("Dock", _agentId),
    m_dockTargetId(_dockTargetId),
    m_b(Behavior_Arrive(m_dockTargetId, 10, _agentId))
{
    m_dockTarget = VehicleManager::getInstance()->getVehicle(m_dockTargetId);
//    m_b = new Behavior_Arrive(m_dockTargetId, 2, _agentId);
}

Behavior_Dock::~Behavior_Dock()
{
    //    delete m_b;
}

std::string Behavior_Dock::getDockTargetId()
{
    return m_dockTargetId;
}

Vehicle *Behavior_Dock::getDockTarget()
{
    return m_dockTarget;
}

Steering Behavior_Dock::apply()
{
    return m_b.apply();
}

void Behavior_Dock::setAgent(std::string _agentId)
{
    Behavior::setAgent(_agentId);
    m_b.setAgent(_agentId);
}

} // namespace Behaviors
} // namespace Delivery
} // namespace LIMoSim
