#ifndef UIDATAMANAGER_H
#define UIDATAMANAGER_H

#include <map>
#include <memory>
#include <vector>

#include "uidataentrymobile.h"
#include "uidataentrystatic.h"

#include "LIMoSim/world/uidata.h"

namespace LIMoSim {

class Vehicle;

namespace UI {

namespace Data {


typedef std::map<std::string, UIDataEntryMobile_ptr> VehicleAgentData;
typedef std::map<std::string, UIDataEntryStatic_ptr> StaticAgentData;
typedef std::map<std::string, uint> ConnectionCountMap;
typedef std::pair<std::string, std::string> Connection;
typedef std::vector<Connection> Connections;

class UIDataManager: public UIData
{
public:
    static UIDataManager *getInstance();

    const VehicleAgentData& getVehiclesData();
    UIDataEntryMobile_ptr getVehicleData(std::string _vehicleId);
    const StaticAgentData& getStaticNodesData();
    UIDataEntryStatic_ptr getStaticNodeData(std::string _staticNodeId);
    bool getConnectionStatus(std::string _nodeId);
    std::vector<std::string> getConnectedNodeIds();
    Connections getNodeConnections();

    bool isStaticNodeRegistered(std::string _staticNodeId);
    bool isVehicleRegistered(std::string _vehicleId);
    bool isConnectionRegistered(std::string _node1, std::string _node2);

    void registerVehicles();
    void registerStaticNode(std::string _nodeId, Vector3d _position, Orientation3d _orientation);
    void registerStaticNode(std::string _nodeId, Vector3d _position, Orientation3d _orientation, Shape_Ptrs _shapes);

    void registerConnection(std::string _node1, std::string _node2);
    void unregisterConnection(std::string _node1, std::string _node2);
    void unregisterConnectionsOf(std::string _node);
    void clearConnections();

private:
    /**
     * @brief m_vehicles
     * Map of vehicle nodes ids to the UIDataEntries which shall be used
     * to draw them.
     * Vehicle nodes are nodes, the mobility of which is initialized at simulation begin
     * and handled by LIMoSim during simulation.
     */
    VehicleAgentData m_vehicles;
    /**
     * @brief m_staticNodes
     * Map of static node ids to the UIDataEntries which shall be used
     * to draw them.
     * Static nodes are nodes which get their position initialized at simulation begin
     * and are never to be changed again during the simulation.
     * The mobility of static nodes is therefore not handled by LIMoSim
     */
    StaticAgentData m_staticNodes;
    /**
     * @brief m_connectionStatus
     * Map that keeps track of the number of connections of each node.
     * A node is seen as connected if the number of connections is bigger than 1.
     * A node is seen as unconnected if the number of connections is 0.
     */
    ConnectionCountMap m_connectionStatus;
    /**
     * @brief m_connections
     * List of Connections between nodes which are to be drawn.
     * A connection is represented by a pair formed of the node ids
     * of the involved nodes
     */
    Connections m_connections;

    UIDataManager();
    ~UIDataManager();
    void registerVehicle(Vehicle* _vehicle);
    void unregisterVehicle(std::string _vehicleId);
};

} // namespace Data

} // namespace UI

} // namespace LIMoSim

#endif // UIDATAMANAGER_H
