#ifndef TRUCKDELIVERY_H
#define TRUCKDELIVERY_H

#include "deterministicpath.h"
#include "LIMoSim/mobility/routing/tsp.h"

#include "LIMoSim/mobility/deliverytruck/deliverytruck.h"
#include "LIMoSim/simulation/eventhandler.h"
#include "LIMoSim/world/worldutils.h"
#include "demo/deliverytypedefs.h"

namespace LIMoSim {

using namespace  mobility::car;
using namespace  world::utils;
using namespace  delivery::typedefs;

enum TruckDeliveryState {NONE=-1, STOPPED=0, DELIVERING, UNLOADING, LAUNCHING, RECOVERING};

class TruckDelivery : public DeterministicPath, public EventHandler
{
public:
    TruckDelivery(DeliveryTruck *_truck);
    Vector3d getcurrentWaypoint();
    Endpoint *getNextEndpoint();
    TruckDeliveryState getState();
    virtual void launchUAVOnSite(Endpoint* m_endpoint);
    virtual void launchUAVEnRoute(EnRouteDelivery _delivery);
    void stopDelivery();
    void resumeDelivery(double _delay = 0);
    void startFirstDelivery();
    DroneInteractionMode getDroneInteractionMode();
    void setDroneInteractionMode(DroneInteractionMode _mode);
    uint getAvailableDroneCount();

    void checkEnRouteDroneHandling();
    std::vector<std::string> createDoneDeliveryList(TSPD_Solution _tspd, BuildingToEndpoint _buildings);
    std::vector<std::string> createTruckDeliveryList(TSPD_Solution _tspd, LIMoSim::world::utils::BuildingToEndpoint _buildings);
    /**
     * @brief determineNextLaneSegment
     * Finds out which lane segment must be encountered next
     * in order to guarantee gate routing an delivery plan consistency.
     * If the lane segment following the one the truck is currently on
     * aligns with gate routing and delivery consistency then it is
     * returned.
     * If not the first lane segment in opposite direction on the same
     * lane segment the truck is currently on is returned instead.
     * @param _nextNode
     * @return the next lane the truck should follow to maintain gate
     * routing and delivery plan consistency.
     */
    LaneSegment* determineNextLaneSegment(Node *_nextNode);
    double distanceToNextEndpoint();
    bool endpointIsAhead(Endpoint* _start, Endpoint* _target);
    int getEndpointIndexInRoute(Endpoint *_endpoint);
    bool isFirstDeliveryCompleted();
    void printTspRoute();
    void printDeliveryList();

    bool isIntersectionBeforeEndpoint(Endpoint *_start, Endpoint *_next, RoadSegment *_srcSegment);



    // StrategicModel interface
public:
    void initialize();
    void handleEvent(Event *_event);
    void handleNodeReached(Node *_node);
    void handleGateReached(Gate *_gate, Intersection *_intersection, LaneSegment *_lane, LaneSegment *_nextLaneSegment);
    virtual void handleDeliveryTargetReached(std::string _targetId, Endpoint *_endpoint);

protected:
    DeliveryTruck *p_truck;
    TruckDeliveryState m_state;
    DroneInteractionMode m_droneInteractionMode;
    TSP_Solution m_tspSolution;
    TSPD_Solution m_tspd;
    std::vector<Endpoint*> m_endpoints;
    std::vector<EnRouteDelivery> m_droneDeliveries;


    void decrementDroneCount();
    void incrementDroneCount();

private:
    Gate *m_start, *m_end;
    Endpoint* m_nextEnpoint;

    BuildingToEndpoint m_buildingsToEndpoints;

    uint m_tspRouteIdx;

    bool m_firstDeliveryCompleted = false;
    uint m_availableDroneCount=0;
};

} // namespace LIMoSim

#endif // TRUCKDELIVERY_H
