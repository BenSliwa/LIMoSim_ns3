#include "uidatamanager.h"

#include <algorithm>
#include <memory>
#include <functional>

#include "defaultshapes.h"

#include "LIMoSim/world/vehiclemanager.h"

// TODO:
// Register vehicle on creation in VehicleManager

namespace LIMoSim {

namespace UI {

namespace Data {
UIDataManager *UIDataManager::getInstance()
{
    static UIDataManager instance;
    return &instance;
}

const VehicleAgentData &UIDataManager::getVehiclesData()
{
    return m_vehicles;
}

UIDataEntryMobile_ptr UIDataManager::getVehicleData(std::string _vehicleId)
{
    if (m_vehicles.count(_vehicleId)) {
        return m_vehicles.at(_vehicleId);
    } else {
        std::cerr << "retrieving ui data of unregistered vehicle \""
                  << _vehicleId <<"\" failed!" << std::endl;
        return std::shared_ptr<UIDataEntryMobile>(nullptr);
    }
}

const StaticAgentData &UIDataManager::getStaticNodesData()
{
    return m_staticNodes;
}

UIDataEntryStatic_ptr UIDataManager::getStaticNodeData(std::string _staticNodeId)
{
    if (m_staticNodes.count(_staticNodeId)) {
        return m_staticNodes.at(_staticNodeId);
    } else {
        std::cerr << "retrieving ui data of unregistered static node \""
                  << _staticNodeId << "\" failed!" <<std::endl;
        return nullptr;
    }
}

bool UIDataManager::getConnectionStatus(std::string _nodeId)
{
    return m_connectionStatus.count(_nodeId) && m_connectionStatus.at(_nodeId);
}

std::vector<std::string> UIDataManager::getConnectedNodeIds()
{
    std::vector<std::string> nodeIds;

    for (auto pair: m_connectionStatus) {
        if (pair.second) {
            nodeIds.push_back(pair.first);
        }
    }
    return nodeIds;
}

Connections UIDataManager::getNodeConnections()
{
    Connections connections;
    for (auto connection : m_connections) {
        connections.push_back(connection);
    }
    return connections;
}

bool UIDataManager::isStaticNodeRegistered(std::string _staticNodeId)
{
    return m_staticNodes.count(_staticNodeId);
}

bool UIDataManager::isVehicleRegistered(std::string _vehicleId)
{
    return m_vehicles.count(_vehicleId);
}

bool UIDataManager::isConnectionRegistered(std::string _node1, std::string _node2)
{
    auto it = std::find_if(m_connections.begin(),
                 m_connections.end(),
                 [_node1, _node2] (Connection c) {
        return c == Connection(_node1, _node2) || c == Connection(_node2, _node1);
    });

    return it != m_connections.end();
}

void UIDataManager::registerVehicles()
{
    VehicleMap vehiclesMap = VehicleManager::getInstance()->getVehicles();

    std::for_each(
                vehiclesMap.begin(),
                vehiclesMap.end(),
                [&](VehicleMap::value_type entry) {
                    if (!m_vehicles.count(entry.first)) {
                        if (entry.second) {
                            registerVehicle(entry.second);
                        }
                    }
                }
    );
}

void UIDataManager::registerStaticNode(std::string _nodeId, Vector3d _position, Orientation3d _orientation)
{
    m_staticNodes.insert(
                StaticAgentData::value_type(
                    _nodeId,
                    std::make_shared<UIDataEntryStatic>(UIDataEntryStatic(
                        _nodeId,
                        _position,
                        _orientation
                    ))
                    ));
}

void UIDataManager::registerStaticNode(std::string _nodeId, Vector3d _position, Orientation3d _orientation, Shape_Ptrs _shapes)
{
    m_staticNodes.insert(
                StaticAgentData::value_type(
                    _nodeId,
                    std::make_shared<UIDataEntryStatic>(UIDataEntryStatic(
                        _nodeId,
                        _position,
                        _orientation,
                        _shapes
                    ))
                    ));
}

void UIDataManager::registerConnection(std::string _node1, std::string _node2)
{
    if (!isConnectionRegistered(_node1, _node2)) {
        if (m_connectionStatus.count(_node1)) {
            m_connectionStatus.at(_node1)++;
        } else {
            m_connectionStatus[_node1] = 1;
        }

        if (m_connectionStatus.count(_node2)) {
            m_connectionStatus.at(_node2)++;
        } else {
            m_connectionStatus[_node2] = 1;
        }

        m_connections.push_back(Connection(_node1, _node2));
    }
}

void UIDataManager::unregisterConnection(std::string _node1, std::string _node2)
{

    auto it = std::find_if(m_connections.begin(),
                 m_connections.end(),
                 [_node1, _node2] (Connection c) {
        return c == Connection(_node1, _node2) || c == Connection(_node2, _node1);
    });

    if (it != m_connections.end()) {
        m_connections.erase(it);

        if (m_connectionStatus.count(_node1) && m_connectionStatus.at(_node1)) {
            m_connectionStatus.at(_node1)--;
        } else {
            // This should not happen.
            std::cout << "Warning: cannot reduce connection count for node" << _node1 << "further."  << std::endl;
        }

        if (m_connectionStatus.count(_node2) && m_connectionStatus.at(_node2)) {
            m_connectionStatus.at(_node2)--;
        } else {
            // This should not happen.
            std::cout << "Warning: cannot reduce connection count for node" << _node2 << "further."  << std::endl;
        }
    }
}

void UIDataManager::unregisterConnectionsOf(std::string _node)
{
    auto it = std::find_if(m_connections.begin(),
                 m_connections.end(),
                 [_node] (Connection c) {
        return c.first == _node || c.second == _node;
    });

    while (it != m_connections.end()) {

        std::string node1 = it->first;
        std::string node2 = it->second;
        m_connections.erase(it);

        if (m_connectionStatus.count(node1) && m_connectionStatus.at(node1)) {
            m_connectionStatus.at(node1)--;
        } else {
            // This should not happen.
            std::cout << "Warning: cannot reduce connection count for node" << node1 << "further."  << std::endl;
        }
        if (m_connectionStatus.count(node2) && m_connectionStatus.at(node2)) {
            m_connectionStatus.at(node2)--;
        } else {
            // This should not happen.
            std::cout << "Warning: cannot reduce connection count for node" << node2 << "further."  << std::endl;
        }

        // lookup next connection involving _node
        it = std::find_if(m_connections.begin(),
                         m_connections.end(),
                         [_node] (Connection c) {
                return c.first == _node || c.second == _node;
            });
    }
}

void UIDataManager::clearConnections()
{
    m_connectionStatus.clear();
    m_connections.clear();
}

UIDataManager::UIDataManager()
{
// Test connection status tracking
//    std::cout << "step 1" << std::endl;
//    registerConnection("1", "2");
//    std::cout << "connection count for node: " << 1 << m_connectionStatus.at("1") << std::endl;
//    std::cout << "connection count for node: " << 2 << m_connectionStatus.at("2") << std::endl;

//    std::cout << "step 2" << std::endl;
//    registerConnection("1", "3");
//    std::cout << "connection count for node: " << 1 << m_connectionStatus.at("1") << std::endl;
//    std::cout << "connection count for node: " << 2 << m_connectionStatus.at("2") << std::endl;
//    std::cout << "connection count for node: " << 3 << m_connectionStatus.at("3") << std::endl;

//    std::cout << "step 3" << std::endl;
//    registerConnection("3", "2");
//    std::cout << "connection count for node: " << 1 << m_connectionStatus.at("1") << std::endl;
//    std::cout << "connection count for node: " << 2 << m_connectionStatus.at("2") << std::endl;
//    std::cout << "connection count for node: " << 3 << m_connectionStatus.at("3") << std::endl;

//    std::cout << "step 4" << std::endl;
//    registerConnection("3", "4");
//    std::cout << "connection count for node: " << 1 << m_connectionStatus.at("1") << std::endl;
//    std::cout << "connection count for node: " << 2 << m_connectionStatus.at("2") << std::endl;
//    std::cout << "connection count for node: " << 3 << m_connectionStatus.at("3") << std::endl;
//    std::cout << "connection count for node: " << 4 << m_connectionStatus.at("4") << std::endl;

//    std::cout << "step 5" << std::endl;
//    unregisterConnection("2", "6");
//    std::cout << "connection count for node: " << 1 << m_connectionStatus.at("1") << std::endl;
//    std::cout << "connection count for node: " << 2 << m_connectionStatus.at("2") << std::endl;
//    std::cout << "connection count for node: " << 3 << m_connectionStatus.at("3") << std::endl;
//    std::cout << "connection count for node: " << 4 << m_connectionStatus.at("4") << std::endl;

//    std::cout << "step 6" << std::endl;
//    unregisterConnection("1", "3");
//    std::cout << "connection count for node: " << 1 << m_connectionStatus.at("1") << std::endl;
//    std::cout << "connection count for node: " << 2 << m_connectionStatus.at("2") << std::endl;
//    std::cout << "connection count for node: " << 3 << m_connectionStatus.at("3") << std::endl;
//    std::cout << "connection count for node: " << 4 << m_connectionStatus.at("4") << std::endl;

//    std::cout << "step 7" << std::endl;
//    unregisterConnection("3", "2");
//    std::cout << "connection count for node: " << 1 << m_connectionStatus.at("1") << std::endl;
//    std::cout << "connection count for node: " << 2 << m_connectionStatus.at("2") << std::endl;
//    std::cout << "connection count for node: " << 3 << m_connectionStatus.at("3") << std::endl;
//    std::cout << "connection count for node: " << 4 << m_connectionStatus.at("4") << std::endl;

//    std::cout << "step 8" << std::endl;
//    unregisterConnection("3", "4");
//    std::cout << "connection count for node: " << 1 << m_connectionStatus.at("1") << std::endl;
//    std::cout << "connection count for node: " << 2 << m_connectionStatus.at("2") << std::endl;
//    std::cout << "connection count for node: " << 3 << m_connectionStatus.at("3") << std::endl;
    //    std::cout << "connection count for node: " << 4 << m_connectionStatus.at("4") << std::endl;
}

UIDataManager::~UIDataManager()
{

}

void UIDataManager::registerVehicle(Vehicle *_vehicle)
{
    std::function <Vector3d()> positionGetter = std::bind(&Vehicle::getPosition, _vehicle);
    std::function <Orientation3d()> orientationGetter = std::bind(&Vehicle::getOrientation, _vehicle);
    m_vehicles.insert(
                VehicleAgentData::value_type(
                    _vehicle->getId(),
                    std::make_shared<UIDataEntryMobile>(UIDataEntryMobile(
                        _vehicle->getId(),
                        positionGetter,
                        orientationGetter,
                        defaultVehicleShape(_vehicle)
                                                            ))));
}

void UIDataManager::unregisterVehicle(std::string _vehicleId)
{
    if (m_vehicles.count(_vehicleId)){
        m_vehicles.erase(_vehicleId);
    }
}

} // namespace Data

} // namespace UI

} // namespace LIMoSim
