#ifndef TSP_H
#define TSP_H

#include <map>

#include "LIMoSim/world/road/endpoint.h"
#include "LIMoSim/mobility/routing/types.h"
#include "LIMoSim/world/worldtypes.h"

namespace LIMoSim {


class Intersection;
class Routable;
class Road;
class Gate;

using namespace mobility::routing::types;
using namespace world::types;

//struct IntersectionDiscovery {
//    Intersection *intersection = nullptr;
//    /**
//     * @brief segment
//     * Road segment through which the intersection
//     * should be left to reach the endpoint
//     */
//    RoadSegment *segment = nullptr;

//    Gate *gate;
//    double cost = 0;
//};



struct IntersectionRoutingEntry {
    Intersection *src = nullptr;
    Intersection *dst = nullptr;
    RoadSegment *srcSegmentIn = nullptr;
    Gate *srcGateOut = nullptr;
    Gate *dstGateOut = nullptr;
    Route gateRoute;
    double srcCost = 0;
    double dstCost = 0;
    double irtCost = 0;
    double cost =0;
};

struct EndpointRoutingEntry {
    Endpoint *src;
    Endpoint *dest;
    IntersectionRoutingEntry routing;
    EndpointRoutingEntry(){}
    EndpointRoutingEntry( Endpoint* _src,
            Endpoint *_dest,
            IntersectionRoutingEntry _routing):
        src(_src),
        dest(_dest),
        routing(_routing){}
};

typedef std::vector<uint> Path;
typedef std::vector<Path> Paths;

/**
 * @brief EndpointRoutingTable
 * The routing table which holds the routing data
 * which is valid for one specific endpoint
 */
typedef std::map<Endpoint*, EndpointRoutingEntry> EndpointRoutingTable;
/**
 * @brief EndpointRoutingTables
 * A map of all the routing tables which are valid for each
 * graph endpoint
 */
typedef std::map<Endpoint*, EndpointRoutingTable> EndpointRoutingTables;


struct TSP_Solution {
    Paths routes;
    EndpointRoutingTables routing;
    Endpoints endpoints;
};


struct EnRouteDelivery {
    /**
     * @brief nextTruckDelivery
     * Index of next truck delivery in TSPD truck route
     */
    uint nextTruckDelivery;
    /**
     * @brief launchDistance
     * Distance from the UAV delivery target
     * at which the UAV must be launched.
     */
    double launchDistance;
    /**
     * @brief deliveryTarget
     * Index of the delivery target in TSPD UAV route
     */
    uint deliveryTarget;
};

struct EnRouteKey {
    /**
     * @brief truckDeliveryIndex
     * Index of next truck delivery in TSPD truck route
     */
    uint truckDeliveryIndex;
    /**
     * @brief uavDeliveryIndex
     * Index of the delivery target in TSPD UAV route
     */
    uint uavDeliveryIndex;
};
inline bool operator<(const EnRouteKey& k1, const EnRouteKey& k2) {
    if (k1.truckDeliveryIndex != k2.truckDeliveryIndex) {
        return k1.truckDeliveryIndex < k2.truckDeliveryIndex;
    } else {
        return k1.uavDeliveryIndex < k2.uavDeliveryIndex;
    }
}

/**
 * @brief EnRouteLaunchRegistry
 * Maps the index of the next truck delivery in TSPD truck route
 * to the
 */
typedef std::map<EnRouteKey, EnRouteDelivery> EnRouteLaunchRegistry;

struct TSPD_Solution {
    Path truckRoute;
    EndpointRoutingTables routing;
    Endpoints endpoints;
    /**
     * @brief launchRegistry
     * Maps each endpoint index to the UAV-Endpoints
     * for which the UAVs must be launched at the node.
     */
    std::map<uint, std::vector<uint>> launchRegistry;
    std::vector<uint> UAVEndpoints;
    std::vector<EnRouteDelivery> enRouteDeliveries;
    /**
     * @brief recoveryConstraints
     * To each endpoint index, maps a list of
     * indexes referring to endpoints who must
     * have been handled before it.
     * Currently only used for UAV endpoints
     * in order to model the the fact that the truck
     * must wait at some endpoints for some UAVs
     * to come back because the next endpoints are out
     * of range for the UAV returning from its last mission.
     */
    std::map<uint, std::vector<uint>> recoveryConstraints;
};

typedef std::vector<Routable*> RoutingGraph;

//using namespace world::utils;

namespace mobility {
namespace routing {

class TSP
{
public:
    TSP();

    TSPD_Solution addDroneSupport(TSP_Solution _tsp, uint _droneCount, uint _tspRouteIndex=0);
    TSP_Solution solve(Endpoints _endpoints, uint _start);

    bool checkRangeForDroneTrip(Endpoint* _from, Endpoint* _to);
    bool checkRangeForDroneUsage(Endpoint* _from, Endpoint *_main, Endpoint* _to);
    std::map<uint, std::vector<uint> > createPrecedenceConstraints(Path _tspPath,
            Path _tspdPath,
            Endpoints _endpoints, std::vector<uint> _UavEndpoints, uint _droneCapacity,
            std::map<uint, std::vector<uint>> _launchPlan);
    void drawSolution(TSP_Solution _tsp, uint _routeIdx=0);
    void drawSolution(TSPD_Solution _tspd);

    double estimateDroneOperationCost(
            uint _launchIdx,
            uint _deliveryIdx,
            uint _regroupIdx,
            TSP_Solution _tspSolution,
            uint _tspRouteIndex
            );
    double estimateTruckOperationCost(uint _launchIdx,
            uint _deliveryIdx,
            uint _regroupIdx,
            TSP_Solution _tspSolution,
            uint _tspRouteIndex);

protected:
    EndpointRoutingEntry getEndpointTravelCost(Endpoint* p_start, Endpoint* p_end, RoutingGraph& _graph);
    double getIntersectionTravelCost(IntersectionDiscovery &isec1,
                                     IntersectionDiscovery &isec2,
                                     RoutingGraph &_graph);
    double getIntersectionTravelCost(IntersectionRoutingEntry &_isecRouting, RoutingGraph &_graph);
//    Road* getNextExplorationRoad(Road *_road, Node *_node) const;
    std::vector<IntersectionRoutingEntry> generateIntersectionRouting(
            IntersectionDiscovery& _isec1,
            IntersectionDiscovery& _isec2
            );
public:
//    IntersectionDiscovery getNextIntersectionForEndpoint(
//            Endpoint* _endpoint,
//            const bool &_forward = true
//            ) const;
protected:
    void performIntersectionDiscovery(Endpoint *p_endpoint, IntersectionDiscovery& _iSecDiscSelf, IntersectionDiscovery& _iSecDiscFwd, IntersectionDiscovery& _iSecDiscBwd);


private:
    Endpoints m_endpoints;
    RoutingGraph m_graph;
    std::vector<std::vector<double>> m_costs;
    EndpointRoutingTables m_routingTables;
    double uav_range_max = 2500.0;
    double uav_range_min = 30;
    bool displayIntersectionEdges = false;

    void initCosts();
    void printCosts();




};

} // namespace routing
} // namespace mobility
} // namespace LIMoSim

#endif // TSP_H
