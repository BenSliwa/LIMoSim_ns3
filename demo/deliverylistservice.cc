#include "deliverylistservice.h"

#include <algorithm>
#include <iostream>

#include "LIMoSim/world/road/endpoint.h"

namespace LIMoSim {
namespace delivery {

DeliveryListService *DeliveryListService::getInstance()
{
    static DeliveryListService instance;
    return &instance;
}

void DeliveryListService::addDeliveryTarget(std::string _buildingId)
{
    if (!m_isDefinitive) {
        m_deliveryList.push_back(_buildingId);
    }
}

void DeliveryListService::addDeliveryTargets(std::vector<std::string> _buildingIds)
{
//    if (!m_isDefinitive) {
//        m_deliveryList.insert(m_deliveryList.end(),
//                              _buildingIds.begin(),
//                              _buildingIds.end());
//    }
}

void DeliveryListService::addDroneDeliveryTargets(StringVector _buildingIds)
{
    if (!m_isDefinitive) {
        m_droneDeliveryList.insert(m_droneDeliveryList.end(),
                              _buildingIds.begin(),
                              _buildingIds.end());
        m_deliveryMonitor.registerDeliveries(_buildingIds);
    }
}

void DeliveryListService::addTruckDeliveryTargets(StringVector _buildingIds)
{
    if (!m_isDefinitive) {
        m_truckDeliveryList.insert(m_truckDeliveryList.end(),
                              _buildingIds.begin(),
                              _buildingIds.end());
        m_deliveryMonitor.registerDeliveries(_buildingIds);
    }
}

bool DeliveryListService::allDelivered()
{
    return this->allTruckTargetsDelivered() && this->allDroneTargetsDelivered();
}

bool DeliveryListService::allDroneTargetsDelivered()
{
    bool result = m_nextDroneTarget == m_droneDeliveryList.end();
    return result;
}

bool DeliveryListService::allTruckTargetsDelivered()
{
    return m_nextTruckTarget == m_truckDeliveryList.end() && m_pendingDeliveries.empty();
}

const StringVector DeliveryListService::getDeliveryList()
{
    return m_deliveryList;
}

std::string DeliveryListService::getDeliveryStatusColor(std::string _buildingId,
                                                        std::string _notListed)
{
    auto inTruckList = std::find(m_truckDeliveryList.begin(),
                            m_truckDeliveryList.end(),
                            _buildingId);
    auto inDroneList = std::find(m_droneDeliveryList.begin(),
                            m_droneDeliveryList.end(),
                            _buildingId);

    StringVectorIterator listPos, currentListPos;
    StringVector list;
    if (inTruckList != m_truckDeliveryList.end()) {
        list = m_truckDeliveryList;
        listPos = inTruckList;
        currentListPos = m_nextTruckTarget;
    } else if (inDroneList != m_droneDeliveryList.end()) {
        list = m_droneDeliveryList;
        listPos = inDroneList;
        currentListPos = m_nextDroneTarget;
    } else {
        return _notListed;
    }

    auto inPending = std::find(m_pendingDeliveries.begin(),
                               m_pendingDeliveries.end(),
                               _buildingId);
    if (inPending != m_pendingDeliveries.end()) {
        return "orange";
    } else {
        bool delivered = std::distance(listPos, currentListPos) > 0;
        return delivered ? "green" : "red";
    }
}

std::string DeliveryListService::getNextDeliveryTarget()
{
    if (m_nextTarget != m_deliveryList.end()) {
        std::string nextTarget = (*m_nextTarget++);
        m_pendingDeliveries.push_back(nextTarget);
        return nextTarget;
    } else {
        return "";
    }
//    return m_nextTarget != m_deliveryList.end() ?
//                (*m_nextTarget++):
//                "";
}

std::string DeliveryListService::getNextDroneDeliveryTarget()
{
    if (m_nextDroneTarget != m_droneDeliveryList.end()) {
        std::string nextTarget = (*m_nextDroneTarget++);
        m_pendingDeliveries.push_back(nextTarget);
        return nextTarget;
    } else {
        return "";
    }
}

std::string DeliveryListService::getNextTruckDeliveryTarget()
{
    if (m_nextTruckTarget != m_truckDeliveryList.end()) {
        std::string nextTarget = (*m_nextTruckTarget++);
        m_pendingDeliveries.push_back(nextTarget);
        return nextTarget;
    } else {
        return "";
    }

}

size_t DeliveryListService::getPendingCount()
{
    return m_pendingDeliveries.size();
}

TSP_Solution DeliveryListService::getTSP()
{
    return m_tspSolution;
}

bool DeliveryListService::isStatusUpdatePending()
{
    return m_pendingStatusUpdate;
}

void DeliveryListService::makeListDefinitive()
{
    m_isDefinitive = true;
    m_nextTruckTarget = m_truckDeliveryList.begin();
    m_nextDroneTarget = m_droneDeliveryList.begin();
    m_deliveryList.clear();
    m_deliveryList.insert(
                m_deliveryList.end(),
                m_truckDeliveryList.begin(),
                m_truckDeliveryList.end()
                );
    m_deliveryList.insert(
                m_deliveryList.end(),
                m_droneDeliveryList.begin(),
                m_droneDeliveryList.end()
                );
    m_deliveryListProgress.insert(
                m_deliveryListProgress.begin(),
                m_deliveryList.begin(),
                m_deliveryList.end()
                );
    m_nextTarget = m_deliveryList.begin();
}

void DeliveryListService::notifyDelivery(std::string _buildingId)
{
    auto it = std::find(m_pendingDeliveries.begin(),
                        m_pendingDeliveries.end(),
                        _buildingId);
    if (it != m_pendingDeliveries.end()) {
        std::cout << "erasing target " << _buildingId << " from delivery list" << std::endl;
        m_pendingDeliveries.erase(it);
        auto it1 = std::find(m_deliveryListProgress.begin(), m_deliveryListProgress.end(), _buildingId);
        m_deliveryListProgress.erase(it1);
        std::cout << "remaining deliveries: " << m_deliveryListProgress.size() << std::endl;
        m_pendingStatusUpdate = true;
        m_deliveryMonitor.notifyDelivery(_buildingId);
    }
}

void DeliveryListService::notifyStatusUpdateApplied()
{
    m_pendingStatusUpdate = false;
}

void DeliveryListService::storeTSP(TSP_Solution _endpoints)
{
    m_tspSolution = _endpoints;
}

DeliveryListService::DeliveryListService():
    m_isDefinitive(false)
{

}

} // namespace Demo
} // namespace LIMoSim
