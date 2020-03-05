#include "tsp.h"


#include "algorithm"
#include <bits/stdc++.h>

#include "LIMoSim/mobility/routing/routingutils.h"
#include "LIMoSim/mobility/routing/dijkstra.h"
#include "LIMoSim/mobility/routing/linkernighan.h"

#include "LIMoSim/world/world.h"
#include "LIMoSim/world/worldutils.h"


#include "ui/data/uidatamanager.h"
#include "ui/data/defaultshapes.h"

#include "utils/vector.h"


namespace LIMoSim {
namespace mobility {
namespace routing {

TSP::TSP():
    m_graph(World::getInstance()->buildRoutingGraph())
{

}

TSPD_Solution TSP::addDroneSupport(TSP_Solution _tsp, uint _droneCount, uint _tspRouteIndex)
{

    TSPD_Solution tspd;
    tspd.routing = _tsp.routing;
    tspd.endpoints = _tsp.endpoints;
    Path tspPath = _tsp.routes.at(_tspRouteIndex);

    // Initialize launch registry
    for(uint i = 0; i < tspd.endpoints.size(); i++) {
        tspd.launchRegistry[i] = std::vector<uint>();
    }

    // for each endpoint in tsp path
    for (uint i = 0, skip = 0; i < tspPath.size(); i=i+skip+1) {
        uint droneCap = _droneCount;
        skip = 0;
        tspd.truckRoute.push_back(tspPath.at(i));

        // No launching allowed at the last 2 nodes.
        // These are (in order):
        // - last delivery
        // - depot
        if (i >= tspPath.size()-2) {
            continue;
        }

        bool launching = true;

        std::vector<uint> UAVEndpoints;
        // for each endpoint j in the rest of the tsp path
        uint j = 1;
        for (; j < tspPath.size() - i - 1 && droneCap > 0; j++) {

            // check if endpoint i+j is in uav reach
            // of endpoint i and endpoint i+j+1
            // TODO: Also check for alignment of the three nodes
            bool inDroneRange = checkRangeForDroneUsage(
                        _tsp.endpoints.at(tspPath.at(i)),
                        _tsp.endpoints.at(tspPath.at(i+j)),
                        _tsp.endpoints.at(tspPath.at(i+j+1))
                        );
            double droneCost = estimateDroneOperationCost(i, j, i+j+1, _tsp, _tspRouteIndex);
            double truckCost = estimateTruckOperationCost(i, i+j, i+j+1, _tsp, _tspRouteIndex);
            if ( inDroneRange && droneCost < truckCost) {
                // if so:

                // mark endpoint i as launch endpoint with 1 more occurence
                tspd.launchRegistry.at(tspPath.at(i)).push_back(tspPath.at(i+j));
                // mark endpoint i+j as uav endpoint
                tspd.UAVEndpoints.push_back(tspPath.at(i+j));
                // decrement drone capacity
                droneCap--;
                // increment skip counter to skip node i+j in outer loop
                skip++;
            } else {
                // if not:
                launching = false;
                // stop looking for drone endpoint from endpoint i
                // break inner loop
                break;
            }
        }
    }

    // Generate precedence constraints for drone recovery
    std::cout <<"computing recovery constraints" << std::endl;
    tspd.recoveryConstraints = createPrecedenceConstraints(
                tspPath,
                tspd.truckRoute,
                tspd.endpoints,
                tspd.UAVEndpoints,
                _droneCount,
                tspd.launchRegistry
                );

    std::cout <<"recovery constraints: " << std::endl;
    for(auto k : tspd.recoveryConstraints) {
        std::cout << k.first << ": ";
        for (auto v : k.second) {
            std::cout << v << " ";
        }
        std::cout << std::endl;
    }

//    for (auto v : tspd.UAVEndpoints) {
//        std::cout << v << " ";
//    }
//    std::cout << std::endl;

    std::cout <<"computing enroute recovery constraints" << std::endl;
    // Iterate on the truck route to preserve order in en route delivery.
    for (uint truckRouteIndex = 1; truckRouteIndex < tspd.truckRoute.size(); truckRouteIndex++) {
        auto truckEndpointIndex = tspd.truckRoute.at(truckRouteIndex);

        if (tspd.recoveryConstraints.count(truckEndpointIndex)) {
            auto recoveredDeliveries = tspd.recoveryConstraints.at(truckEndpointIndex);

            for (auto uavDeliveryTargetIndex : recoveredDeliveries) {
                EnRouteDelivery enRouteDelivery;
                enRouteDelivery.nextTruckDelivery = static_cast<uint>(LIMoSim::utils::vector::indexOf(tspd.truckRoute, truckEndpointIndex, 1));

                // Find the uav endpoint among the list of uav endpoints
                auto uavEndpointIt = std::find(
                            tspd.UAVEndpoints.begin(),
                            tspd.UAVEndpoints.end(),
                            uavDeliveryTargetIndex);


                // Find the index of the uav endpoint among the list of uav endpoints
                enRouteDelivery.deliveryTarget = uavEndpointIt != tspd.UAVEndpoints.begin() ?
                            static_cast<uint>(std::distance(
                                                  tspd.UAVEndpoints.begin(),
                                                  uavEndpointIt)
                                              )
                          :
                            0;
//                enRouteDelivery.deliveryTarget = static_cast<uint>(std::distance(
//                            tspd.UAVEndpoints.begin(),
//                            std::find(
//                                tspd.UAVEndpoints.begin(),
//                                tspd.UAVEndpoints.end(),
//                                uavDeliveryTargetIndex)
//                            ));std::cout <<"TSP::AddDroneSupport before 3" << std::endl;
//                if (tspd.truckRoute.size() == 2) {

//                }
//                IntersectionRoutingEntry routing = tspd.routing.at(
//                            tspd.endpoints.at(tspd.truckRoute[enRouteDelivery.nextTruckDelivery-1])
//                            ).at(
//                            tspd.endpoints.at(tspd.truckRoute.at(enRouteDelivery.nextTruckDelivery))
//                            ).routing;std::cout <<"TSP::AddDroneSupport before 4" << std::endl;

                Gate *startGate, *endGate;
                Endpoint *deliveryEndpoint = tspd.endpoints.at(tspd.UAVEndpoints.at(enRouteDelivery.deliveryTarget));
                // TODO: later check along the route if there is one
                // which gates are the most suitable for the computation
                // for now just use endpoints
//                if (routing.gateRoute.size() > 0) {
                    Vector3d startPos = tspd.endpoints.at(tspd.truckRoute[enRouteDelivery.nextTruckDelivery-1])->getNode()->getPosition();
                    Vector3d endPos = tspd.endpoints.at(tspd.truckRoute[enRouteDelivery.nextTruckDelivery])->getNode()->getPosition();
                    double startDistance = (startPos - deliveryEndpoint->getNode()->getPosition()).norm();
                    double endDistance = (endPos - deliveryEndpoint->getNode()->getPosition()).norm();
                    enRouteDelivery.launchDistance = (startDistance + endDistance)/2;
                    tspd.enRouteDeliveries.push_back(enRouteDelivery);
//                    Route route = routing.gateRoute;
//                    startGate = dynamic_cast<Gate*>(route.at(0));
//                    endGate = dynamic_cast<Gate*>(route.at(route.size()-1));
//                } else {
                    // only one gate on path from nextTruckDelivery-1 to nextTruckDelivery
                    // then use source and dest gate from routing instead
                    // Question: would this not be ok in all cases?
//                    startGate = routing.srcGateOut;
//                    endGate = routing.dstGateOut;
//                    double startDistance = (startGate->getEndpoint()->getNode()->getPosition() - deliveryEndpoint->getNode()->getPosition()).norm();
//                    double endDistance = (endGate->getEndpoint()->getNode()->getPosition() - deliveryEndpoint->getNode()->getPosition()).norm();
//                    enRouteDelivery.launchDistance = (startDistance + endDistance)/2;
//                    tspd.enRouteDeliveries.push_back(enRouteDelivery);
//                }
            }
        }
    }

    return tspd;
}

TSP_Solution TSP::solve(Endpoints _endpoints, uint _start)
{
    std::cout << "Start TSP solving " << std::endl;
    m_endpoints = _endpoints;
    size_t endpointCount = m_endpoints.size();

    // init the costs of travelling between each endpoint pair
    this->initCosts();
    std::cout << "endpoint routing cost table:" << std::endl;
    this->printCosts();

    TSP_Solution solution;
    solution.routing = m_routingTables;
    solution.endpoints.insert(solution.endpoints.begin(), _endpoints.begin(), _endpoints.end());

    std::vector<uint> ids(m_endpoints.size());
    std::iota(std::begin(ids), std::end(ids), 0);
    std::vector<std::vector<double>> costs = m_costs;
    LinKernighan lk(m_costs, ids);
    lk.optimizeTour();
    solution.routes.push_back(lk.getCurrentTour());
    // Finalize LK solution by adding tour start
    solution.routes.at(0).insert(solution.routes.at(0).begin(), 0);

    return solution;

    /**
     * Implementation based on https://www.geeksforgeeks.org/traveling-salesman-problem-tsp-implementation/
     */

    // store endpoints indexes except start endpoint
    std::vector<uint> vertex;
    std::vector<uint> minPath;

    minPath.resize(endpointCount);
    for (uint i = 0; i < endpointCount; i++) {
        if (i != _start) {
            vertex.push_back(i);
        }
    }

    // store minimum weight hamiltonian cycle
    uint minPathWeight = INT_MAX;
    int perm = 0;
    do {
        std::cout << "checking permuation: " << ++perm << std::endl;
        // store current Path weight(cost)
        uint currentPathWeight = 0;

        // compute current path weight
        uint k = _start;
        for (size_t i = 0; i < vertex.size(); i++) {
            currentPathWeight += m_costs[k][vertex[i]];
            k = vertex[i];
        }
        currentPathWeight += m_costs[k][_start];

        // update minimum and store minimum paths
        if (minPathWeight >= currentPathWeight) {
            minPath.clear();

            // retrieve current path
            minPath.push_back(_start);
            for (auto it=vertex.begin(); it!=vertex.end(); it++) {
                minPath.push_back(*it);
            }
            minPath.push_back(_start);

            // update minimum if required
            if (minPathWeight > currentPathWeight) {
               solution. routes.clear();
                minPathWeight = currentPathWeight;
            }


            solution.routes.push_back(minPath);
        }

    } while (next_permutation(vertex.begin(), vertex.end()));


    return solution;
}

bool TSP::checkRangeForDroneTrip(Endpoint *_from, Endpoint *_to)
{
    double distance = (_to->getNode()->getPosition() - _from->getNode()->getPosition()).norm();
    bool ok = (uav_range_min < distance && distance < uav_range_max);
    return ok;
}

bool TSP::checkRangeForDroneUsage(Endpoint *_from, Endpoint *_main, Endpoint *_to)
{

    double forth = (_main->getNode()->getPosition() - _from->getNode()->getPosition()).norm();
    double back = (_to->getNode()->getPosition() - _main->getNode()->getPosition()).norm();
    bool ok = (
                uav_range_min < forth && forth < uav_range_max &&
                uav_range_min < back && back < uav_range_max
                );
    return ok;
}

std::map<uint, std::vector<uint>> TSP::createPrecedenceConstraints(
        Path _tspPath,
        Path _tspdPath,
        Endpoints _endpoints,
        std::vector<uint> _UavEndpoints,
        uint _droneCapacity,
        std::map<uint, std::vector<uint> > _launchPlan
        )
{

    std::map<uint, std::vector<uint>> constraints;
    auto start = _tspPath.begin();
    auto end = _tspPath.begin();
    size_t tspdSize = _tspdPath.size();

    // For each endpoint on the tsp-d route
    for (uint i = 1; i < tspdSize; i++) {
        // Determine which UAV missions may still be pending.
        // These are the missions missing from the tsp-d route
        // when comparing to the tsp route
        start = end;
        end =  tspdSize > 2 ? std::find(start, _tspPath.end(), _tspdPath.at(i)) : _tspPath.end()-1;
        size_t incomingUAVCount = static_cast<size_t>(std::abs(std::distance(start, end))) -1;
        std::vector<uint> UAVEndpoints;

        if (incomingUAVCount) {
             UAVEndpoints.resize(incomingUAVCount);
             std::copy(start+1, end, UAVEndpoints.begin());
        } else {
            continue;
        }

        // Impl2: the next truck endpoint is the recovery point
        for (uint it = 0; it < UAVEndpoints.size(); it++) {
            if (!constraints.count(_tspdPath.at(i))) {
                constraints[_tspdPath.at(i)] = std::vector<uint>();
            }
            constraints.at(_tspdPath.at(i)).push_back(UAVEndpoints.at(it));

        }


        // Impl1: the last truck endpoint in uav reach is the recovery point
        // For each endpoint k on the tsp-route starting from i (the next truck endpoint) onwards
//        uint k = 0;
//        for(; k < _tspdPath.size() - i && incomingUAVCount > 0; k++) {

//            std::vector<uint> clearedEndpoints;

//            // For each UAVEndpoint for which mission shall start from endpoint i-1
//            for (uint it = 0; it < UAVEndpoints.size(); it++) {

//                // Check if the endpoint i+k+1 within UAV-reach from the UAV endpoint
//                if (
//                        (i+k == _tspdPath.size() - 1) ||
//                        !checkRangeForDroneTrip(
////                            _endpoints.at(_tspPath.at(UAVEndpoints.at(it))),
//                            _endpoints.at(UAVEndpoints.at(it)),
//                            _endpoints.at(_tspdPath.at(i+k+1))
//                            )
//                        ){
//                    // If not register a precedence constraint stating that
//                    // the UAV must return from it's mission endpoint
//                    // prior to the truck leaving endpoint i+k
//                    if (!constraints.count(_tspdPath.at(i+k))) {
//                        constraints[_tspdPath.at(i+k)] = std::vector<uint>();
//                    }
//                    constraints.at(_tspdPath.at(i+k)).push_back(UAVEndpoints.at(it));
//                    clearedEndpoints.push_back(UAVEndpoints.at(it));
//                    incomingUAVCount--;
//                }
//            }

//            // Remove the returned UAVs from the incoming list
//            for(auto it : clearedEndpoints) {
//                std::remove(UAVEndpoints.begin(), UAVEndpoints.end(), it);
//            }
//        }

        // Remaing incoming UAVs may return at further endpoints.
        // Depending on the drone capacity and further launching plan.

        // Determine the furthest the remaing UAV can return based on launch contraints
//        if (incomingUAVCount){
//            // For each of the next endpoints
//            for (uint j = 0; j < _tspdPath.size() - i - 1 && incomingUAVCount > 0; j++) {
//                // Check the launching balance
//                if (_launchPlan.at(_tspdPath.at(i+j)).size() - (_droneCapacity-incomingUAVCount) > 0) {
//                    // if deficient, the difference in amount of drones must return at endpoint i+j
//                    std::vector<uint> clearedEndpoints;
//                    const ulong required = std::min(static_cast<unsigned long>(0), _launchPlan.at(_tspdPath.at(i+j)).size() - (_droneCapacity-incomingUAVCount));
//                    for(uint r = 0; r < required; r++) {
//                        if (!constraints.count(_tspdPath.at(i+j))) {
//                            constraints[_tspdPath.at(i+j)] = std::vector<uint>();
//                        }
//                        constraints.at(_tspdPath.at(i+j)).push_back(UAVEndpoints.at(r));
//                        clearedEndpoints.push_back(UAVEndpoints.at(r));
//                        incomingUAVCount--;
//                    }
//                    // Remove the returned UAVs from the incoming list
//                    for(auto it : clearedEndpoints) {
//                        std::remove(UAVEndpoints.begin(), UAVEndpoints.end(), it);
//                    }
//                }

//            }
//        }

        // Determine the furthest the remaining UAVs
        // may return to.
//        if (incomingUAVCount) {
//            uint l = i + ( (k+i == _tspdPath.size() -1) ? k-1 : k );
//            for(auto it = UAVEndpoints.begin(); it != UAVEndpoints.end(); it++) {
//                if (!constraints.count(_tspdPath.at(l))) {
//                    constraints[_tspdPath.at(l)] = std::vector<uint>();
//                }
//                constraints.at(_tspdPath.at(l)).push_back(*it);
//            }
//        }


    }
    return constraints;
}

void TSP::drawSolution(TSP_Solution _tsp, uint _routeIdx)
{
    UI::Data::UIDataManager *uidm = UI::Data::UIDataManager::getInstance();

    Path path = _tsp.routes.at(_routeIdx);
    // Show truck delivery endpoints
    for (auto truckNode : path) {
        uidm->registerStaticNode(_tsp.endpoints.at(truckNode)->getNode()->getId(),
                                 _tsp.endpoints.at(truckNode)->getNode()->getPosition() + Vector3d(0,0,50),
                                 Orientation3d(90),
                                 UI::Data::defaultGateShape());
    }

    // Show truck node connections
    for (uint i = 1; i < path.size(); i++) {
        uidm->registerConnection(
                    _tsp.endpoints.at(path.at(i))->getNode()->getId(),
                    _tsp.endpoints.at(path.at(i-1))->getNode()->getId()
                    );
    }
}

void TSP::drawSolution(TSPD_Solution _tspd)
{

    UI::Data::UIDataManager *uidm = UI::Data::UIDataManager::getInstance();

    // Show truck delivery endpoints
    for (auto truckNode : _tspd.truckRoute) {
        uidm->registerStaticNode(_tspd.endpoints.at(truckNode)->getNode()->getId(),
                                 _tspd.endpoints.at(truckNode)->getNode()->getPosition() + Vector3d(0,0,50),
                                 Orientation3d(90),
                                 UI::Data::defaultGateShape());
    }
    // Show UAV delivery endpoints
    for (auto UAVNode : _tspd.UAVEndpoints) {
        uidm->registerStaticNode(_tspd.endpoints.at(UAVNode)->getNode()->getId(),
                                 _tspd.endpoints.at(UAVNode)->getNode()->getPosition() + Vector3d(0,0,50),
                                 Orientation3d(90),
                                 UI::Data::defaultGateShape("yellow"));
    }

    // Show truck node connections
    for (uint i = 1; i < _tspd.truckRoute.size(); i++) {
        uidm->registerConnection(
                    _tspd.endpoints.at(_tspd.truckRoute.at(i))->getNode()->getId(),
                    _tspd.endpoints.at(_tspd.truckRoute.at(i-1))->getNode()->getId()
                    );
    }

    // Show UAV node connections
    for (auto it : _tspd.launchRegistry) {
        uint launchNode = it.first;
        std::vector<uint> targets = it.second;
        for (uint targetIdx : targets) {
            uidm->registerConnection(
                        _tspd.endpoints.at(targetIdx)->getNode()->getId(),
                        _tspd.endpoints.at(launchNode)->getNode()->getId()
                        );
        }
    }
    for (auto it : _tspd.recoveryConstraints) {
        uint recoveryNode = it.first;
        std::vector<uint> sources = it.second;
        for (uint sourcetIdx : sources) {
            uidm->registerConnection(
                        _tspd.endpoints.at(sourcetIdx)->getNode()->getId(),
                        _tspd.endpoints.at(recoveryNode)->getNode()->getId()
                        );
        }
    }
}

double TSP::estimateDroneOperationCost(uint _launchIdx, uint _deliveryIdx, uint _regroupIdx, TSP_Solution _tspSolution, uint _tspRouteIndex)
{
    Path path = _tspSolution.routes.at(_tspRouteIndex);
    Endpoints endpoints = _tspSolution.endpoints;
    Endpoint* launch = endpoints.at(path.at(_launchIdx));
    Endpoint* delivery = endpoints.at(path.at(_deliveryIdx));
    Endpoint* regroup = endpoints.at(path.at(_regroupIdx));
    static double ascendAndDescentCost = 0; //100;
    double deliverycost = ascendAndDescentCost +
            (launch->getNode()->getPosition() - delivery->getNode()->getPosition()).norm();
    double regroupCost = ascendAndDescentCost +
            (delivery->getNode()->getPosition() - regroup->getNode()->getPosition()).norm();
    static double deliveryWaitingTimeForDrone_sec = 0; // 30;
    return deliverycost + regroupCost + deliveryWaitingTimeForDrone_sec;
}

double TSP::estimateTruckOperationCost(uint _launchIdx, uint _deliveryIdx, uint _regroupIdx, TSP_Solution _tspSolution, uint _tspRouteIndex)
{
    Path path = _tspSolution.routes.at(_tspRouteIndex);
    Endpoints endpoints = _tspSolution.endpoints;
    Endpoint* launch = endpoints.at(path.at(_launchIdx));
    Endpoint* delivery = endpoints.at(path.at(_deliveryIdx));
    Endpoint* regroup = endpoints.at(path.at(_regroupIdx));
    Route deliveryRoute = _tspSolution.routing.at(launch).at(delivery).routing.gateRoute;
    double deliveryCost =  _tspSolution.routing.at(launch).at(delivery).routing.cost;
    Route regroupRoute = _tspSolution.routing.at(delivery).at(regroup).routing.gateRoute;
    double regroupCost = _tspSolution.routing.at(launch).at(delivery).routing.cost;
    static double deliveryWaitingTimeForTruck_sec = 160;
    return deliveryCost + regroupCost + deliveryWaitingTimeForTruck_sec;
}

EndpointRoutingEntry TSP::getEndpointTravelCost(Endpoint *p_start, Endpoint *p_end, RoutingGraph &_graph)
{
    /*
     * Retrieve all the possible intersection associated with each endpoint
     * For each endpoint a maximum of 2 intersections is possible
     * and one intersection is retrieved if the endpoint is at an intersection.
     */

    IntersectionDiscovery start_self, start_fwd, start_bwd, end_self, end_fwd, end_bwd;

    // Retrieve intersections associated to start endpoint
    performIntersectionDiscovery(p_start, start_self, start_fwd, start_bwd);

    // Retrieve intersections associated with end endpoint
    performIntersectionDiscovery(p_end, end_self, end_fwd, end_bwd);


    // Create all the possible intersection pairs
    std::vector<double> costs;
    std::vector<std::pair<IntersectionDiscovery, IntersectionDiscovery>> isec_pairs;
    std::vector<IntersectionRoutingEntry> routingEntries;
    if (!start_self.intersection && !end_self.intersection) {
        // if neither of the endpoints is at an intersection
        isec_pairs = {
            std::make_pair(start_fwd, end_fwd),
            std::make_pair(start_fwd, end_bwd),
            std::make_pair(start_bwd, end_fwd),
            std::make_pair(start_bwd, end_bwd)
        };
    } else if (!start_self.intersection && end_self.intersection) {
        // if only the end endpoint is at an intersection
        isec_pairs = {
            std::make_pair(start_fwd, end_self),
            std::make_pair(start_bwd, end_self)
        };
    } else if (start_self.intersection && !end_self.intersection) {
        // if only the start endpoint is at an intersection
        isec_pairs = {
            std::make_pair(start_self, end_fwd),
            std::make_pair(start_self, end_bwd)
        };
    } else if (start_self.intersection && end_self.intersection) {
        // If both endpoints are at intersections
        isec_pairs = {
            std::make_pair(start_self, end_self)
        };
//        return EndpointRoutingEntry(p_start, p_end, start_self, end_self, cost);
    }

    // Compute the cost of each intersection pair
//    costs.resize(isec_pairs.size());
    for (size_t i = 0; i < isec_pairs.size(); i++) {
        auto isec_pair = isec_pairs.at(i);
        auto entries = generateIntersectionRouting(isec_pair.first, isec_pair.second);
        for (auto& entry : entries) {
            getIntersectionTravelCost(entry, _graph);
            costs.push_back(entry.cost);
        }
        routingEntries.insert(routingEntries.end(), entries.begin(), entries.end());
//        costs[i] = getIntersectionTravelCost(isec_pair.first, isec_pair.second, _graph);
    }

    // pick the minimun
    if (costs.size() == 0) {
        std::cerr << "TSP::getEndpointTravelCost No costs were calculated!" << std::endl;
        exit(1);
    }
    auto min_iter = std::min_element(costs.begin(), costs.end());
    auto routingEntry = routingEntries.at(std::distance(std::begin(costs), min_iter));
    return EndpointRoutingEntry(p_start, p_end, routingEntry);

}

double TSP::getIntersectionTravelCost(IntersectionDiscovery &isec1, IntersectionDiscovery &isec2, RoutingGraph &_graph)
{
    if (isec1.intersection == isec2.intersection) {
        // if same intersection
        if (isec1.segment == isec2.segment) {
            // and same road segment
            return std::abs(isec1.cost - isec2.cost);
        } else {
            // and different road segment
            return std::abs(isec1.cost + isec2.cost);
        }
    } else {
        // if distinct intersections
        Dijkstra dijkstra;
        dijkstra.setGraph(_graph);
        std::vector<Routable*> path;
//        std::vector<Routable*> path = dijkstra.computeShortestPath(
//                    isec1.intersection->getOutGateOfFirstDifferenSegment(isec1.segment),
//                    isec2.intersection->getOutGateOfFirstDifferenSegment(isec2.segment));
        double cost = isec1.cost + isec2.cost;
        for (size_t i = 0; i < path.size() - 1 ; i++) {
            Routable *current = path.at(i), *next = path.at(i+1);
            cost += current->getCost(next);
        }
        return cost;
    }
}

double TSP::getIntersectionTravelCost(IntersectionRoutingEntry &_isecRouting, RoutingGraph &_graph)
{
    if (_isecRouting.src == _isecRouting.dst) {
        // if same intersection
        if (_isecRouting.srcSegmentIn->getRoad()->getName() == _isecRouting.dstGateOut->getEndpoint()->getOwner()->getRoad()->getName()) {
            // and same road segment
            _isecRouting.irtCost = 0;
            _isecRouting.cost = std::abs(_isecRouting.srcCost - _isecRouting.dstCost);
            return _isecRouting.cost;
        } else {
            // and different road segment
            _isecRouting.irtCost = 0;
            _isecRouting.cost = _isecRouting.srcCost + _isecRouting.dstCost;
            return _isecRouting.cost;
        }
    } else {
        // if distinct intersections
        Dijkstra dijkstra;
        dijkstra.setGraph(_graph);
        _isecRouting.gateRoute = dijkstra.computeShortestPath(
                    _isecRouting.srcGateOut,
                    _isecRouting.dstGateOut);
        double cost = 0;
        cost = routing::utils::computeRouteCost(_isecRouting.gateRoute);
        _isecRouting.irtCost = cost;
        _isecRouting.cost = _isecRouting.srcCost + _isecRouting.irtCost + _isecRouting.dstCost;
        return cost;
    }
}

//Road *TSP::getNextExplorationRoad(Road *_road, Node *_node) const
//{
//    // as the node is no intersection, it has exactly two roads
//    std::vector<Road*> roads = World::getInstance()->filterRoads(_node->getWays());
//    if(roads.at(0)==_road)
//        return roads.at(1);
//    else if(roads.at(1)==_road)
//        return roads.at(0);
//    return nullptr;
//}

std::vector<IntersectionRoutingEntry> TSP::generateIntersectionRouting(IntersectionDiscovery &_isec1, IntersectionDiscovery &_isec2)
{
    /*
     * For each intersection pair, N intersection routing entries are generated.
     * N being the number of outgates of the intersection posing as routing source
     */
    std::vector<IntersectionRoutingEntry> routingEntries;
    std::vector<Gate*> outGates;
//    std::vector<Gate*> outGates = _isec1.intersection->getOtherOutGates(
//                _isec1.intersection->getOutGate(_isec1.segment)
//                );
    if (outGates.empty() || _isec1.cost == 0.0) {
        outGates.push_back(_isec1.intersection->getOutGate(_isec1.segment));
    }
    for (Gate *g : outGates) {
        IntersectionRoutingEntry entry;
        entry.src = _isec1.intersection;
        if (_isec1.cost == 0.0) {
            entry.srcSegmentIn = _isec1.intersection->getOutSegmentForGate(g);
        } else {
            entry.srcSegmentIn = _isec1.segment;
        }
        entry.srcSegmentIn = _isec1.segment;
        entry.srcGateOut = g;
        entry.srcCost = _isec1.cost;
        entry.dst = _isec2.intersection;
        entry.dstGateOut = _isec2.intersection->getOutGate(_isec2.segment);
        entry.dstCost = _isec2.cost;
        routingEntries.push_back(entry);
    }

    return routingEntries;

}

//IntersectionDiscovery TSP::getNextIntersectionForEndpoint(Endpoint *_endpoint, const bool &_forward) const
//{
//    Intersection *intersection = nullptr;
//    RoadSegment *segment = _endpoint->getOwner();
//    Road *road = segment->getRoad();
//    IntersectionDiscovery disc;
//    disc.cost = 0;

//    World *p_world = World::getInstance();

//    int inc = _forward ? 1 : -1;


//    Node *node;
//    if (_forward) {
//        node =  _endpoint->getOwner()->getToEndpoint()->getNode();
//    } else {
//        node =  _endpoint->getOwner()->getFromEndpoint()->getNode();
//    }

//    if (
//            (_forward && _endpoint->getNode() != _endpoint->getOwner()->getToEndpoint()->getNode()) ||
//            (!_forward && _endpoint->getNode() != _endpoint->getOwner()->getFromEndpoint()->getNode())
//            ){
//        disc.cost += (node->getPosition() - _endpoint->getNode()->getPosition()).norm();
//    }


//    intersection = p_world->getIntersection(node);
//    while (!intersection) {
//        int index = segment->getIndex() + inc;
//        segment = road->getSegmentWithIndex(index);

//        if(!segment) // switch the roads - the current node is either start or end of the other way
//        {
//            // determine the next road and the next segment
//            road = getNextExplorationRoad(road, node);
//            if(road->getEndNode()==node)
//            {
//                segment = road->getLastSegment();
//                inc = -1;
//            }
//            else if(road->getStartNode()==node)
//            {
//                segment = road->getFirstSegment();
//                inc = 1;
//            }
//        }
//        disc.cost += segment->getLength();
//        node = segment->getOtherNode(node);
//        intersection = p_world->getIntersection(node);
//    };

//    disc.intersection = intersection;
//    disc.segment = segment;

//    return disc;
//}

void TSP::performIntersectionDiscovery(Endpoint *p_endpoint, IntersectionDiscovery &_iSecDiscSelf, IntersectionDiscovery &_iSecDiscFwd, IntersectionDiscovery &_iSecDiscBwd)
{
    Intersection *endpointAsIntersection = World::getInstance()->getIntersection(p_endpoint->getNode());
    if (endpointAsIntersection) {
        // If endpoint is intersection
        _iSecDiscSelf.intersection = endpointAsIntersection;
        _iSecDiscSelf.cost = 0;
        _iSecDiscSelf.segment = p_endpoint->getOwner();

        // display intersection
        if (displayIntersectionEdges){
            UI::Data::UIDataManager::getInstance()->registerStaticNode(std::string("intersec_endpoint ") + endpointAsIntersection->getNode()->getId(),
                endpointAsIntersection->getNode()->getPosition(),
                Orientation3d(0),
                UI::Data::defaultGateShape());
        }
    } else {
        // If not retreive the 2 possible intersections associated to the endpoint
        _iSecDiscFwd = world::utils::getNextIntersectionForEndpoint(p_endpoint);
        _iSecDiscBwd = world::utils::getNextIntersectionForEndpoint(p_endpoint, false);

        // display intersection
        if (displayIntersectionEdges){
            UI::Data::UIDataManager::getInstance()->registerStaticNode(std::string("intersec_start ") + _iSecDiscFwd.intersection->getNode()->getId(),
                _iSecDiscFwd.intersection->getNode()->getPosition(),
                Orientation3d(0),
                UI::Data::defaultGateShape("blue"));
            UI::Data::UIDataManager::getInstance()->registerStaticNode(std::string("intersec_end ") + _iSecDiscBwd.intersection->getNode()->getId(),
                _iSecDiscBwd.intersection->getNode()->getPosition(),
                Orientation3d(0),
                UI::Data::defaultGateShape("red"));
        }
    }
}

void TSP::initCosts()
{
    m_costs.clear();
    m_costs.resize(m_endpoints.size());

    for (size_t i = 0; i < m_endpoints.size(); i++) {
        m_costs.at(i).resize(m_endpoints.size());
        m_routingTables[m_endpoints.at(i)] = EndpointRoutingTable();
    }

    for (size_t i = 0; i < m_endpoints.size(); i++) {
        std::vector<double> i_costs;
        for (size_t j = 0; j < m_endpoints.size(); j++) {
            if(i != j){
                Endpoint *e1 = m_endpoints[i], *e2 = m_endpoints[j];
                EndpointRoutingEntry e = getEndpointTravelCost(e1, e2, m_graph);
                m_routingTables[e1][e2] = e;
//                EndpointRoutingEntry _e = e;
//                _e.src = e.dest;
//                _e.dest = e.src;
//                _e.isecSrc = e.isecDst;
//                _e.isecDst = e.isecSrc;
//                m_routingTables[m_endpoints.at(j)][m_endpoints.at(i)] = _e;
                m_costs[i][j] = e.routing.cost;
//                m_costs[j][i] = e.cost;
            } else {
                m_costs[i][j] = 0;
            }
        }
    }
}

void TSP::printCosts()
{
    for (size_t i = 0; i < m_costs.size(); i++) {
        std::cout << m_endpoints.at(i)->getNode()->getId() << "\t";
        for (size_t j =0; j < m_costs.at(i).size(); j++) {
            std::cout << m_costs[i][j] << "\t";
        }
        std::cout << std::endl;
    }
}

} // namespace routing
} // namespace mobility
} // namespace LIMoSim
