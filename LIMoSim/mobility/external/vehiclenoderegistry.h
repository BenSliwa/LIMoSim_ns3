#ifndef VEHICLENODEREGISTRY_H
#define VEHICLENODEREGISTRY_H

#include <map>

namespace LIMoSim {
namespace Mobility{
namespace External {

using NodeId = std::string;
using VehicleId = std::string;

/**
 * @brief The VehicleNodeRegistry class
 * Registry to retrieve LIMoSim vehicle Id from node id in coupled simulator
 */
class VehicleNodeRegistry
{
public:
    static VehicleNodeRegistry* getInstance();
    VehicleId getVehicleId(NodeId _nodeId);
    bool isNodeRegistered(NodeId _nodeId);
    void registerVehicleNode(NodeId _nodeId, VehicleId _vehicleId);
    void clearRegistry();

private:
    std::map<NodeId, VehicleId> m_vehicleNodes;

    VehicleNodeRegistry();
};

} // namespace External
} // namespace Mobility
} // namespace LIMoSim

#endif // VEHICLENODEREGISTRY_H
