#include "vehiclenoderegistry.h"

#include <iostream>

namespace LIMoSim {
namespace Mobility {
namespace External{

VehicleNodeRegistry *VehicleNodeRegistry::getInstance()
{
    static VehicleNodeRegistry _instance;
    return &_instance;
}

VehicleId VehicleNodeRegistry::getVehicleId(NodeId _nodeId)
{
    if (m_vehicleNodes.count(_nodeId)) {
        return m_vehicleNodes.at(_nodeId);
    } else {
        return "";
    }
}

bool VehicleNodeRegistry::isNodeRegistered(NodeId _nodeId)
{
    return m_vehicleNodes.count(_nodeId);
}

void VehicleNodeRegistry::registerVehicleNode(NodeId _nodeId, VehicleId _vehicleId)
{
    if (m_vehicleNodes.count(_nodeId)) {
        std::cerr << "VehicleNodeRegistry: cannot register nodeId" << _nodeId << ". Already in use." << std::endl;
        return;
    }
    m_vehicleNodes[_nodeId] = _vehicleId;
}

void VehicleNodeRegistry::clearRegistry()
{
    m_vehicleNodes.clear();
}
VehicleNodeRegistry::VehicleNodeRegistry()
{

}

} // namespace External
} // namespace Mobility
} // namespace LIMoSim
