#include "truckdelivery.h"

#include <set>
#include <algorithm>

#include "demo/deliverylistservice.h"
#include "demo/deliverysettingsservice.h"

#include "LIMoSim/mobility/routing/dijkstra.h"
#include "LIMoSim/world/world.h"
#include "LIMoSim/utils/map.h"
#include "LIMoSim/utils/vector.h"


#include "ui/data/uidatamanager.h"
#include "ui/data/defaultshapes.h"

namespace LIMoSim {

using namespace utils;

TruckDelivery::TruckDelivery(DeliveryTruck *_truck):
    DeterministicPath (_truck),
    EventHandler (),
    p_truck (_truck),
    m_state(STOPPED)
{
    clearGateData();
    m_droneInteractionMode = static_cast<DroneInteractionMode>(std::get<int>(*delivery::DeliverySettingsService::getInstance()->getDeliverySetting(
                delivery::DeliverySettingsService::DeliverySettingKey::DRONE_INTERACTION_MODE
                )));
}

Vector3d TruckDelivery::getcurrentWaypoint()
{
    RoadPosition info = p_car->getRoadPosition();
    if (info.path.size() > 0 && m_state == DELIVERING) {
        return info.path.at(0)->getEndpoint()->getNode()->getPosition();
    } else {
        return p_car->getPosition();
    }
}

Endpoint *TruckDelivery::getNextEndpoint()
{
    if (m_endpoints.size() > 0) {
        return m_endpoints.front();
    }
    return nullptr;
}

TruckDeliveryState TruckDelivery::getState()
{
    return m_state;
}

void TruckDelivery::launchUAVOnSite(Endpoint *m_endpoint)
{

}

void TruckDelivery::launchUAVEnRoute(EnRouteDelivery _delivery)
{

}

void TruckDelivery::stopDelivery()
{
    m_state = STOPPED;
    scheduleEventDelayed(new Event(0, p_truck, "TruckDelivery::Stop"));
}

void TruckDelivery::resumeDelivery(double _delay)
{
    m_state = DELIVERING;
    scheduleEventDelayed(new Event(_delay, p_truck, "TruckDelivery::Start"));
}

void TruckDelivery::startFirstDelivery()
{

    handleNodeReached(m_endpoints.front()->getNode());
    m_firstDeliveryCompleted = true;
}

DroneInteractionMode TruckDelivery::getDroneInteractionMode()
{
    return m_droneInteractionMode;
}

void TruckDelivery::setDroneInteractionMode(DroneInteractionMode _mode)
{
    m_droneInteractionMode = _mode;
}

uint TruckDelivery::getAvailableDroneCount()
{
    return m_availableDroneCount;
}

void TruckDelivery::checkEnRouteDroneHandling()
{
    if (m_droneInteractionMode == ENROUTE && !m_droneDeliveries.empty()) {

        // determine drone deliveries to be started before reaching the next endpoint
        // on the truck route
        std::vector<EnRouteDelivery> nextDroneDeliveries;
        std::vector<EnRouteDelivery>::iterator nextDroneDeliveryIt = m_droneDeliveries.begin();
        while (
               nextDroneDeliveryIt != m_droneDeliveries.end() &&
               m_tspd.endpoints.at(m_tspd.truckRoute.at(nextDroneDeliveryIt->nextTruckDelivery)) == m_endpoints.front()
               ) {
            nextDroneDeliveries.push_back(*nextDroneDeliveryIt);
            nextDroneDeliveryIt++;
        }

        // Check and eventually perform en route drone handling
        // for each of the scheduled deliveries
        for (EnRouteDelivery nextDroneDelivery : nextDroneDeliveries) {
            Vector3d currentTruckPosition = p_truck->getPosition();
            Endpoint* nextDroneDeliveryEndpoint =  m_tspd.endpoints.at(
                        m_tspd.UAVEndpoints.at(nextDroneDelivery.deliveryTarget)
                        );
            Vector3d deliveryTargetPosition = nextDroneDeliveryEndpoint
                    ->getNode()
                    ->getPosition();
            double currentDistanceToTarget = (currentTruckPosition -
                                              deliveryTargetPosition).norm();

            std::cout << "dist to next uav delivery("<< nextDroneDeliveryEndpoint->getNode()->getId() <<") :"
                      << currentDistanceToTarget
                      << " launch distance: " << nextDroneDelivery.launchDistance
                      << " avail. drones: " << getAvailableDroneCount()
                      << std::endl;

            // If truck in within bounds of lauching distance to next delivery target
            if (
                    (std::abs(
                        currentDistanceToTarget - nextDroneDelivery.launchDistance
                        ) < 20) ||
                    currentDistanceToTarget < nextDroneDelivery.launchDistance

                ) {
                // Trigger en route delivery by stopping truck and starting launching logic
                if (m_state == DELIVERING) {
                   std::cout << "TruckDelivery::checkEnRouteDroneHandling: stop to launch or wait for recovery" << std::endl;
                   this->stopDelivery();
                }
                // If there currently is no UAV available for delivery,
                // the truck should stop and wait for one to return.
                // The launch logic should only be started once
                // there is an available UAV for delivery.
                if (getAvailableDroneCount() > 0 && m_state != RECOVERING){
                    launchUAVEnRoute(nextDroneDelivery);
                    m_droneDeliveries.erase(m_droneDeliveries.begin());
                }
            }
        }
    }
}

std::vector<std::string> TruckDelivery::createDoneDeliveryList(TSPD_Solution _tspd, BuildingToEndpoint _buildings)
{
    Path route = _tspd.UAVEndpoints;
    StringVector buildings;

    // Retrieve the buildings ids of each endpoint
    for(auto endpointIdx : route) {
        Endpoint *endpoint = _tspd.endpoints.at(endpointIdx);
        auto it = map::findByValue(_buildings, endpoint);
        if (it != _buildings.end()) {
            buildings.push_back((*it).first->getId());
        }
    }

    return buildings;
}

std::vector<std::string> TruckDelivery::createTruckDeliveryList(TSPD_Solution _tspd, BuildingToEndpoint _buildings)
{
    Path route = _tspd.truckRoute;
    StringVector buildings;

    // Retrieve the buildings ids of each endpoint
    for(auto endpointIdx : route) {
        Endpoint *endpoint = _tspd.endpoints.at(endpointIdx);
        auto it = map::findByValue(_buildings, endpoint);
        if (it != _buildings.end()) {
            buildings.push_back((*it).first->getId());
        }
    }

    return buildings;
}

LaneSegment *TruckDelivery::determineNextLaneSegment(Node *_nextNode)
{
    if (_nextNode == m_endpoints.front()->getNode() && m_endpoints.size() > 1) {

        Endpoint *current = m_endpoints.front();
        std::cout << "TruckDelivery::determineNextLaneSegment truck delivery next node shall be:"
                  << current->getNode()->getId() << std::endl;

        // Check if redirection needed
        std::cout << "TruckDelivery::determineNextLaneSegment check if u-turn required" << std::endl;

        // Retrieve the next intersection to be encountered
        // in the current drive direction
        // TODO: extract in method without using tsp method
        RoadPosition roadPosition = p_car->getRoadPosition();
        bool forward = !bool(roadPosition.laneSegment->getType());
        IntersectionDiscovery isecdisc = getNextIntersectionForEndpoint(current, forward);
        Node *intersection = isecdisc.intersection->getNode();

        // Check if the delivery endpoint scheduled after the next node to be handled
        // can only be reached after passing an intersection
        bool nextEndpointAfterIntersection = false;
        if (m_endpoints.size() > 1) {
            Endpoint *next = m_endpoints.at(1);
            auto routing = m_tspSolution.routing.at(current).at(next);
            nextEndpointAfterIntersection = isIntersectionBeforeEndpoint(current, next,routing.routing.srcSegmentIn);
        }

        // If the endpoint scheduled after the next node to be handled
        // is only reachable through an intersection
        // and the next intersection in path is not the one detected
        // in drive direction then a u-turn must be made
        // in order for the scheduled intersection to be the one to be handled next
        // in the handleGate method.
        if (nextEndpointAfterIntersection && roadPosition.path.front()->getEndpoint()->getNode() != intersection) {
            std::cout << "TruckDelivery::determineNextLaneSegment trigger u-turn to route back to intersection" << std::endl;
            return roadPosition.laneSegment->getSegment()->getLane((forward ? 1: 0), 0);
        }
        // If the endpoint scheduled after the next node to be handled
        // is on the road section we are on, meaning no intersection must be crossed
        // then check if it is reachable in drive direction.
        // if not turn around.
        else if (!nextEndpointAfterIntersection && !endpointIsAhead(current, m_endpoints.at(1))) {
            std::cout << "TruckDelivery::determineNextLaneSegment trigger u-turn to reach next endpoint on same section" << std::endl;
            return roadPosition.laneSegment->getSegment()->getLane((forward ? 1: 0), 0);
        }
    }
    return p_car->getRoadPosition().laneSegment->getNext();
}

double TruckDelivery::distanceToNextEndpoint()
{
    if (m_endpoints.size() > 0)
        return (m_endpoints.front()->getNode()->getPosition() - p_car->getPosition()).norm();
    return 0;
}

bool TruckDelivery::endpointIsAhead(Endpoint *_start, Endpoint *_target)
{
    // Start from the segment the _start endpoint is on.
    // Navigate in current driving direction untill you find
    // an intersection or the target _endpoint
    RoadSegment *segment = _start->getOwner();
    Road *road = segment->getRoad();
    World *p_world = World::getInstance();
//    Intersection *isec = p_world->getIntersection(_end);
//    int inc = _segment->getIndex() - segment->getIndex() > 0 ? 1 : -1;
//    int inc = _segment->getIndex() > 0 ? -1 : 1;
    int inc = this->p_truck->getRoadPosition().laneSegment->getType() == 0 ? 1 : -1;

    Node * node;
    if (inc > 0) {
        node = _start->getOwner()->getToEndpoint()->getNode();
    } else {
        node = _start->getOwner()->getFromEndpoint()->getNode();
    }

    Intersection * intersection = nullptr;
    intersection = p_world->getIntersection(node);
    while (!intersection) {

        if (
                inc > 0 && segment->getToEndpoint()->getNode() == _target->getNode() ||
                inc <0 && segment->getFromEndpoint()->getNode() == _target->getNode()
                ) {
            return true;
        }

//        if (segment->getToEndpoint() == _target || segment->getFromEndpoint() == _target) {
//            return true;
//        }

        int index = segment->getIndex() + inc;
        segment = road->getSegmentWithIndex(index);

        if(!segment) // switch the roads - the current node is either start or end of the other way
        {
            // determine the next road and the next segment
            // the junction endpoint on the next road must be checked
            // before continuing in the next iteration
//            road = tsp.getNextExplorationRoad(road, node);
            std::vector<Road*> roads = p_world->filterRoads(node->getWays());
            if(roads.at(0)==road)
                road = roads.at(1);
            else if(roads.at(1)==road)
                road = roads.at(0);
            else
                road = nullptr;
            if(road->getEndNode()==node)
            {
                segment = road->getLastSegment();
                inc = -1;
                // check junction endpoint
                if (segment->getToEndpoint() == _target) {
                    return true;
                }
            }
            else if(road->getStartNode()==node)
            {
                segment = road->getFirstSegment();
                inc = 1;
                // check junction endpoint
                if (segment->getFromEndpoint() == _target) {
                    return true;
                }
            }
        }
        node = segment->getOtherNode(node);
        intersection = p_world->getIntersection(node);
    };

    // Check if the searched endpoint is not at the intersection

    return node == _target->getNode();

}

int TruckDelivery::getEndpointIndexInRoute(Endpoint* _endpoint)
{
    Endpoints endpoints = m_tspSolution.endpoints;
    if (!utils::vector::contains(endpoints, _endpoint)) {
        return -1;
    }

    long posInEndpoints = std::distance(endpoints.begin(), std::find(endpoints.begin(), endpoints.end(), _endpoint));

    Path route = m_tspSolution.routes.at(m_tspRouteIdx);
    int posInRoute =  static_cast<int>(std::distance(route.begin(), std::find(route.begin(), route.end(), posInEndpoints)));
    return posInRoute;
}

bool TruckDelivery::isFirstDeliveryCompleted()
{
    return m_firstDeliveryCompleted;
}

void TruckDelivery::printTspRoute()
{
    std::cout << "Computed TSP Route: " << std::endl;
    Path route = m_tspSolution.routes.at(m_tspRouteIdx);
    for (uint endpointIdx : route) {
        Endpoint * endpoint = m_tspSolution.endpoints.at(endpointIdx);
        std::cout << endpoint->getNode()->getId() << ",\n";
    }
}

void TruckDelivery::printDeliveryList()
{
    std::cout << "Delivery list: " << std::endl;
    auto deliveryList = p_truck->getDeliveryList();
    for (auto delivery : deliveryList) {
        std::cout << delivery << std::endl;
    }
    std::cout << "Delivery list end" << std::endl;
}

bool TruckDelivery::isIntersectionBeforeEndpoint(Endpoint *_start, Endpoint *_next, RoadSegment *_srcSegment)
{

    // TODO: Make simpler by asking two questions
    // Through wich segment did i get in this section ?
    // Which one do i reach first?:
    // - the _next endpoint : false
    // - the next intersection : true

    Intersection *intersection = nullptr;
    RoadSegment *segment = _start->getOwner();

    Road *road = segment->getRoad();
    World *p_world = World::getInstance();

    // If the segment just after the intersection has an index higher than 0
    // then it is not the first one but the last.
    // Therefore the search should be going backwards.
    // If the road the segment is on has multiple segments that is
    int inc;
    if (_srcSegment->getRoad()->getSegments().size() > 1) {
        inc = _srcSegment->getIndex() > 0 ? -1 : 1;
    }
    // If not set direction based on the orientation of the segment
    // towards intersection.
    // segment start is at intersection then forward
    // else backwards
    else {
        if (p_world->getIntersection(_srcSegment->getFromEndpoint()->getNode())) {
            inc = 1;
        } else {
            inc = -1;
        }
    }

    Node *node;
    if (inc > 0) {
        node =  _srcSegment->getToEndpoint()->getNode();
    } else {
        node =  _srcSegment->getFromEndpoint()->getNode();
    }
    road = _srcSegment->getRoad();
    segment = _srcSegment;

    intersection = p_world->getIntersection(node);
    while (!intersection) {

        if (segment->getToEndpoint() == _next || segment->getFromEndpoint() == _next) {
            return false;
        }

        int index = segment->getIndex() + inc;
        segment = road->getSegmentWithIndex(index);

        if(!segment) // switch the roads - the current node is either start or end of the other way
        {
            // determine the next road and the next segment
//            road = tsp.getNextExplorationRoad(road, node);
            std::vector<Road*> roads = p_world->filterRoads(node->getWays());
            if(roads.at(0)==road)
                road = roads.at(1);
            else if(roads.at(1)==road)
                road = roads.at(0);
            else
                road = nullptr;
            if(road->getEndNode()==node)
            {
                segment = road->getLastSegment();
                inc = -1;
            }
            else if(road->getStartNode()==node)
            {
                segment = road->getFirstSegment();
                inc = 1;
            }
        }
        node = segment->getOtherNode(node);
        intersection = p_world->getIntersection(node);
    };

    return true;
}

void TruckDelivery::initialize()
{
    std::cout << "TruckDelivery::initialize" << std::endl;

    std::vector<std::string> deliveryList = p_truck->getDeliveryList();

    printDeliveryList();

    m_availableDroneCount = p_truck->getDelivererCount();


    world::utils::Buildings targetedBuildings;
    for (std::string buildingId : deliveryList) {
        targetedBuildings.push_back(World::getInstance()->getBuildings().at(buildingId));
    }


    // Retreive the suitable delivery endpoint for each building
    world::utils::BuildingToEndpoint buildingEndpoints = world::utils::nearestEndpointToBuilding(targetedBuildings);
    m_buildingsToEndpoints = buildingEndpoints;

    // Extract the delivery endpoints
    Endpoints endpoints;
    for (std::string buildingId : deliveryList) {
        endpoints.push_back(buildingEndpoints.at(
                                World::getInstance()->getBuildings().at(buildingId))
                            );
    }

    // remove endpoints duplicates
    Endpoint* start = endpoints.at(0);
    std::set<Endpoint*> s(endpoints.begin(), endpoints.end());
    endpoints.assign(s.begin(), s.end());
    uint startEndpointIndex = static_cast<unsigned int>(std::distance(endpoints.begin(), std::find(endpoints.begin(), endpoints.end(), start)));
    auto e1 = endpoints.at(0);
    endpoints.at(0) = endpoints.at(startEndpointIndex);
    endpoints.at(startEndpointIndex) = e1;
    startEndpointIndex = 0;

    // Model the delivery problem as a TSP problem and search for solutions
    mobility::routing::TSP tsp;
    m_tspSolution = tsp.solve(endpoints, startEndpointIndex);
    std::cout << "TSP routing done" << std::endl;
    m_tspRouteIdx = 0;
    std::cout << "Adding drone support" << std::endl;
    m_tspd = tsp.addDroneSupport(m_tspSolution, p_truck->getDelivererCount(), m_tspRouteIdx);
    std::cout << "Drone support addition done" << std::endl;
    m_droneDeliveries = m_tspd.enRouteDeliveries;
    tsp.drawSolution(m_tspd);
    printTspRoute();

    // Send the delivery plan to the delivery service
    StringVector truckDeliveryTargets = createTruckDeliveryList(m_tspd, buildingEndpoints);
    StringVector droneDeliveryTargets = createDoneDeliveryList(m_tspd, buildingEndpoints);    
    delivery::DeliveryListService *deliveryService = delivery::DeliveryListService::getInstance();
    deliveryService->addTruckDeliveryTargets(truckDeliveryTargets);
    deliveryService->addDroneDeliveryTargets(droneDeliveryTargets);
    deliveryService->makeListDefinitive();

    // Schedule the first delivery to kickstart the delivery state machine
    deliveryService->getNextTruckDeliveryTarget();

    // Store delivery endpoints in strategy
    std::vector<Gate*> path;
    std::vector<Routable*> path_r;
    std::vector<Routable*> graph = World::getInstance()->buildRoutingGraph();

    for (uint idx : m_tspd.truckRoute) {
        m_endpoints.push_back(m_tspSolution.endpoints.at(idx));
    }

    // Retrieve the routing and adapt it to the truck delivery strategy
    std::cout << "construct truck navigation instructions from tspd routing" << std::endl;

    RoadPosition roadPosition;
    if (m_tspd.truckRoute.size() == 2) {
        /*
         * If only 2 targets are present on the truck route,
         * these must refer to the same location: the depot center.
         * In that case, no navigation instructions must be generated
         * as the truck shall not leave the depot center
         *
         * BUG: However, the delivery logic is not completely bullet proof
         * and in often occurs that the truck is set in motion after a drone
         * recovery. This must be fixed by using a robust delivery protocol.
         */
        roadPosition.laneSegment = m_endpoints.at(0)->getOwner()->getLanes(LANE_TYPE::FORWARD).at(0);
        roadPosition.offset_m = 0;
    } else {

        std::vector<Routable*> step;

        for (size_t i = 0; i < m_tspd.truckRoute.size() -1; i++) {
            Endpoint *start, *stop;
            start = m_tspSolution.endpoints.at(m_tspd.truckRoute.at(i));
            stop = m_tspSolution.endpoints.at(m_tspd.truckRoute.at(i+1));
            auto routing = m_tspSolution.routing.at(start).at(stop);
            Gate *gStart,  *gStop;
            gStart = routing.routing.srcGateOut;
            gStop = routing.routing.dstGateOut;

            if (gStart->getEndpoint()->getNode() == gStop->getEndpoint()->getNode()) {

                if (routing.routing.srcSegmentIn->getRoad()->getName() == gStop->getEndpoint()->getOwner()->getRoad()->getName()) {
                    // start and stop are on the same road

                    if (isIntersectionBeforeEndpoint(start,stop,routing.routing.srcSegmentIn)) {
                        // if intersection present between start and stop
                        // who knows how many more there are
                        // use the precomputed routing
                        step = routing.routing.gateRoute;
                        // if the route has no item
                        if (step.size() == 0) {
                            // then add the stop gate to the route.
                            if (routing.routing.dstCost != 0.0) {
                                // but only if the destination is not an intersection
                                path_r.insert(path_r.end(), gStop);
                            }
                        } else {
                            std::vector<Routable*>::iterator stepStart, stepStop;
                            if (routing.routing.srcCost == 0.0 && i == 0) {
                                // if source  of first step is an intersection then remove it from the path
                                // as the truck shall not encounter it anymore
                                stepStart = step.begin()+1;
                            } else {
                                stepStart = step.begin();
                            }
                            if (routing.routing.dstCost == 0.0) {
                                // If destination is an intersection then remove it from the path
                                // as it shall be included in the next path
                                stepStop = step.end()-1;
                            } else {
                                stepStop = step.end();
                            }
                            path_r.insert(path_r.end(), stepStart, stepStop);

        //                    if (routing.routing.srcCost == 0) {
        //                        // if source is an intersection then remove it from the path
        //                        // as the truck shall not encounter it anymore
        //                        path_r.insert(path_r.end(), step.begin()+1, step.end());
        //                    } else {
        //                        path_r.insert(path_r.end(), step.begin(), step.end());
        //                    }
                        }


                    }
                    else {
                        // if no intersection is present
                        // TODO:
                        // For now no gate routing shall be generated for this step.
                        // The truck should just proceed to the start gate of the next step.
                        // If the stop is not in the direction of the next gate
                        // then the u-turn logic shall kick in.
                        // However if the current step is the first one, the u-turn logic must
                        // be triggered manually from this method. This happens further down.
                        // Therefore for it to work in all cases, the handling of the first delivery
                        // must go through the same handling path of the other deliveries
                    }

                } else {
                    // start and stop are on different segments but secant roads
                    if (routing.routing.dstCost != 0.0) {
                        path_r.push_back(gStop);
                    }
                }

            } else {
                step = routing.routing.gateRoute;
                // if the route has no item
                if (step.size() == 0) {
                    // then add the stop gate to the route.
    //                if (routing.routing.dstCost != 0.0) {
    //                    // but only if the destination is not an intersection
    //                    path_r.insert(path_r.end(), gStop);
    //                }
                } else {
                    std::vector<Routable*>::iterator stepStart, stepStop;
                    if (routing.routing.srcCost == 0.0 && i == 0) {
                        // if source  of first step is an intersection then remove it from the path
                        // as the truck shall not encounter it anymore
                        stepStart = step.begin()+1;
                    } else {
                        stepStart = step.begin();
                    }
                    if (routing.routing.dstCost == 0.0) {
                        // If destination is an intersection then remove it from the path
                        // as it shall be included in the next path
                        stepStop = step.end()-1;
                    } else {
                        stepStop = step.end();
                    }
                    path_r.insert(path_r.end(), stepStart, stepStop);

        //            if (routing.routing.srcCost == 0) {
        //                // if source is an intersection then remove it from the path
        //                // as the truck shall not encounter it anymore
        //                path_r.insert(path_r.end(), step.begin()+1, step.end());
        //            } else {
        //                path_r.insert(path_r.end(), step.begin(), step.end());
        //            }
                }

            }
        }
        for(unsigned int i=0; i<path_r.size(); i++)
        {
            Gate *gate = dynamic_cast<Gate*>(path_r.at(i));
            roadPosition.path.push_back(gate);
        }

        // Set the initial direction of truck towards next intersection in routing
        if (getNextIntersectionForEndpoint(m_endpoints.at(0)).intersection->getOutSegmentForGate(roadPosition.path.at(0))) {
            roadPosition.laneSegment = m_endpoints.at(0)->getOwner()->getLanes(LANE_TYPE::FORWARD).at(0);
            roadPosition.offset_m = 0;
        } else {
                roadPosition.laneSegment = m_endpoints.at(0)->getOwner()->getLanes(LANE_TYPE::BACKWARD).at(0);

        }
        p_car->setRoadPosition(roadPosition);

        // Correct initial orientation with respect to the next endpoint in delivery plan
        // is the endpoint after the start enpoint only reachable after an intersection?
        bool nextEndpointAfterIntersection = isIntersectionBeforeEndpoint(
                    m_endpoints.at(0),
                    m_endpoints.at(1),
                    m_tspSolution.routing.at(m_endpoints.at(0)).at(m_endpoints.at(1))
                        .routing.srcSegmentIn
                    );
        if (!nextEndpointAfterIntersection && !endpointIsAhead(m_endpoints.at(0), m_endpoints.at(1))) {
            bool forward = !bool(roadPosition.laneSegment->getType());
            roadPosition.laneSegment->getSegment()->getLane((forward ? 1: 0), 0);
        }
    }

    roadPosition.offset_m = 0;
//    roadPosition.path.erase(roadPosition.path.begin());
    setPath(roadPosition.path);
    p_car->setRoadPosition(roadPosition);

    // In case the next endpoint is the same section
    // but not in next intersection's direction
    // initiate turn.
    if (m_tspd.truckRoute.size() > 2) {
        LaneSegment* nextSegment = determineNextLaneSegment(m_endpoints.front()->getNode());
        if (nextSegment != roadPosition.laneSegment->getNext()) {
            roadPosition.laneSegment = nextSegment;
            p_car->setRoadPosition(roadPosition);
        }
    }

    std::cout<< "Truck delivery initialization done." << std::endl;

//    startFirstDelivery();
//    scheduleEvent(new Event(0, this, "FIRST_DELIVERY"));
}

void TruckDelivery::handleEvent(Event *_event)
{
    // No event handling.
    // For now this strategy only schedules events
    // on the managed agent
    std::string info =_event->getInfo();

    if  (info == "FIRST_DELIVERY") {
        this->startFirstDelivery();
    }
}

void TruckDelivery::handleNodeReached(Node *_node)
{   
    Endpoint *current = m_endpoints.front();

    if (_node == current->getNode()) {
         std::cout << "TruckDelivery::handleNodeReached truck delivery reached node "
                   << current->getNode()->getId() << std::endl;


        auto its = map::findAllByValue(m_buildingsToEndpoints, current);
        for (auto it: its) {
//            if (it != m_buildingsToEndpoints.end()) {
                handleDeliveryTargetReached(it->first->getId(), current);
//            }
        }

        // May be needed later to emulate delivery wait
        //        scheduleEventDelayed(new Event(0, p_truck, "TruckDelivery::Stop"));
        //        scheduleEventDelayed(new Event(5, p_truck, "TruckDelivery::Start"));

        if (
                m_endpoints.size() == 1 &&
                _node == m_tspd.endpoints.at(m_tspd.truckRoute.at(0))->getNode()
    //            !Demo::DeliveryListService::getInstance()->allTruckTargetsDelivered()
                ) {
            // Truck should stop.
            scheduleEventDelayed(new Event(0, p_truck, "TruckDelivery::Stop"));

            if (delivery::DeliveryListService::getInstance()->allDroneTargetsDelivered() && getAvailableDroneCount() == p_truck->getDelivererCount()) {
                Simulation::getInstance()->stop();
            }
        }


        m_endpoints.erase(m_endpoints.begin());
    }


//    if (delivery::DeliveryListService::getInstance()->allDelivered() && getAvailableDroneCount() == p_truck->getDelivererCount()) {
//        Simulation::getInstance()->stop();
//    }

}

void TruckDelivery::handleDeliveryTargetReached(std::string _targetId, Endpoint *_endpoint)
{
    delivery::DeliveryListService::getInstance()->notifyDelivery(_targetId);
    delivery::DeliveryListService::getInstance()->getNextTruckDeliveryTarget();
}

void TruckDelivery::decrementDroneCount()
{
    m_availableDroneCount--;
}

void TruckDelivery::incrementDroneCount()
{
    m_availableDroneCount++;
}

void TruckDelivery::handleGateReached(Gate *_gate, Intersection *_intersection, LaneSegment *_lane, LaneSegment *_nextLaneSegment)
{
    RoadPosition info = p_car->getRoadPosition();
    std::cout << "TruckDelivery::handleGateReached " << _gate->toString() << " " << info.path.size() << std::endl;

    // Get next lane segment on segment to check if specialized handling is
    // required.
    LaneSegment *nextLaneSegment = p_car->getRoadPosition().laneSegment->getNext();

    if (false/* _nextLaneSegment != nullptr && nextLaneSegment != _nextLaneSegment*/) {
        // special gate handling for truck delivery.
        // May be required when the truck must u-turn
        // at an intersection
        std::cout <<"TruckDelivery::handleGateReached using specialized gate handling" << std::endl;
        info.laneSegment = _nextLaneSegment;
        info.offset_m = 0;
        //
        if (_intersection->getOutGates().size() > 1){
            info.path.erase(info.path.begin());
        }
        p_car->setRoadPosition(info);

    } else {
        // keep following current route using deterministic path strategy
        DeterministicPath::handleGateReached(_gate, _intersection, _lane);
    }


    // print the next gate
    if (p_car->getRoadPosition().path.size()>0)
        std::cout << "TruckDelivery::handleGateReached Next gate shall be " << p_car->getRoadPosition().path.front()->toString() << " " <<  p_car->getRoadPosition().path.size() << std::endl;

//    if (_gate == m_end) {
//        // route to delivery node

//    } else {
//        // keep following current route
//        DeterministicPath::handleGateReached(_gate, _intersection, _lane);
//    }


}

} // namespace LIMoSim
