#ifndef DELIVERYLISTSERVICE_H
#define DELIVERYLISTSERVICE_H

#include "LIMoSim/mobility/routing/tsp.h"
#include "LIMoSim/utils/typedefs.h"

#include "deliverymonitor.h"

namespace LIMoSim {

using namespace utils::typedefs;

namespace delivery {

enum DeliveryStatus {NOT_LISTED=0,
                     PENDING,
                     IN_PROGRESS,
                     COMPLETED};

class DeliveryListService
{
public:
    static DeliveryListService* getInstance();
    ~DeliveryListService() {}

    void addDeliveryTarget(std::string _buildingId);
    void addDeliveryTargets(StringVector _buildingIds);
    void addDroneDeliveryTargets(StringVector _buildingIds);
    void addTruckDeliveryTargets(StringVector _buildingIds);
    bool allDelivered();
    bool allDroneTargetsDelivered();
    bool allTruckTargetsDelivered();
    const StringVector getDeliveryList();
    std::string getDeliveryStatusColor(std::string _buildingId,
                                       std::string _notListed="darkgray");
    std::string getNextDeliveryTarget();
    std::string getNextDroneDeliveryTarget();
    std::string getNextTruckDeliveryTarget();
    size_t getPendingCount();
    TSP_Solution getTSP();
    bool isStatusUpdatePending();
    void makeListDefinitive();
    void notifyDelivery(std::string _buildingId);
    void notifyStatusUpdateApplied();

    void storeTSP(TSP_Solution _endpoints);
private:
    DeliveryListService();

    StringVector m_deliveryList;
    StringVector m_deliveryListProgress;
    StringVector m_truckDeliveryList;
    StringVector m_droneDeliveryList;
    StringVector m_pendingDeliveries;
    bool m_isDefinitive;
    bool m_pendingStatusUpdate;
    StringVectorIterator m_nextTarget;
    StringVectorIterator m_nextTruckTarget;
    StringVectorIterator m_nextDroneTarget;
    TSP_Solution m_tspSolution;
    DeliveryMonitor m_deliveryMonitor;

};

} // namespace Demo
} // namespace LIMoSim

#endif // DELIVERYLISTSERVICE_H
