#include "LIMoSim/world/worldutils.h"
#include <algorithm>
#include <functional>
#include <iterator>

#include "building.h"
#include "road/gate.h"
#include "road/intersection.h"
#include "world.h"

#include "ui/data/uidatamanager.h"
#include "ui/data/defaultshapes.h"

#include "LIMoSim/mobility/routing/dijkstra.h"
#include "LIMoSim/utils/vector.h"

namespace LIMoSim{
double controlledDescent(double descentEnd, double descentStart, double descentFinalValue, double descentStartValue, double value, bool symmetric)
{
    if (descentStart < value) {
        return value;
    }

    double mapped = (value - descentEnd) * (descentStartValue-descentFinalValue)/(descentStart-descentEnd) + descentFinalValue;
    mapped = std::max(std::min(descentStartValue, descentFinalValue), mapped);
    return mapped;
}

Vector3d controlledDescent(double descentEnd, double descentStart, double descentFinalValue, double descentStartValue, Vector3d value)
{
    double valueNorm = value.norm();
    if (valueNorm < 1e-4) {
        return 0;
    }
    if (descentStart < valueNorm) {
        return value;
    }

    double mappedNorm = (valueNorm - descentEnd) * (descentStartValue-descentFinalValue)/(descentStart-descentEnd) + descentFinalValue;
    mappedNorm = std::max(std::min(descentFinalValue,descentStartValue), mappedNorm);

    return value.normed() * mappedNorm;
}

double normalizeAngle(double _angle_deg)
{
    while (_angle_deg < 0) {
        _angle_deg += 360;
    }
    while (_angle_deg > 360) {
        _angle_deg -= 360;
    }

    return _angle_deg;
}

namespace world {
namespace utils {


Vector3d getBuildingPosition(Building *p_building)
{
    return p_building->getStartNode()->getPosition();
}



Road *getNextExplorationRoad(Road *_road, Node *_node)
{
    // as the node is no intersection, it has exactly two roads
    std::vector<Road*> roads = World::getInstance()->filterRoads(_node->getWays());
    if(roads.at(0)==_road)
        return roads.at(1);
    else if(roads.at(1)==_road)
        return roads.at(0);
    return nullptr;
}

IntersectionDiscovery getNextIntersectionForEndpoint(Endpoint *_endpoint, const bool &_forward)
{
    Intersection *intersection = nullptr;
    RoadSegment *segment = _endpoint->getOwner();
    Road *road = segment->getRoad();
    IntersectionDiscovery disc;
    disc.cost = 0;

    World *p_world = World::getInstance();

    int inc = _forward ? 1 : -1;


    Node *node;
    if (_forward) {
        node =  _endpoint->getOwner()->getToEndpoint()->getNode();
    } else {
        node =  _endpoint->getOwner()->getFromEndpoint()->getNode();
    }

    if (
            (_forward && _endpoint->getNode() != _endpoint->getOwner()->getToEndpoint()->getNode()) ||
            (!_forward && _endpoint->getNode() != _endpoint->getOwner()->getFromEndpoint()->getNode())
            ){
        disc.cost += (node->getPosition() - _endpoint->getNode()->getPosition()).norm();
    }


    intersection = p_world->getIntersection(node);
    while (!intersection) {
        int index = segment->getIndex() + inc;
        segment = road->getSegmentWithIndex(index);

        if(!segment) // switch the roads - the current node is either start or end of the other way
        {
            // determine the next road and the next segment
            road = getNextExplorationRoad(road, node);
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
        disc.cost += segment->getLength();
        node = segment->getOtherNode(node);
        intersection = p_world->getIntersection(node);
    };

    disc.intersection = intersection;
    disc.segment = segment;

    return disc;
}



size_t hashDeliveryList(StringVector _deliveryList)
{
    const char* const delim = ", ";
    std::ostringstream imploded;
    std::copy(
                _deliveryList.begin(),
                _deliveryList.end(),
                std::ostream_iterator<std::string>(imploded, delim)
                );
    std::string deliveryListString = imploded.str();
    std::hash<std::string> hash_fn;
    return hash_fn(deliveryListString);
}


Gate *nearestGateToBuilding(Building *p_building)
{
    Vector3d buildingPos = getBuildingPosition(p_building);
    std::map<Node*, Intersection*> isecs = World::getInstance()->getIntersections();
    std::map<Gate*, double> gatesDistance;
    // Retrieve the gates forming each intersection
    // and compute the distance of each one the building
    for (const auto& it : isecs) {
        Intersection* isec = it.second;
        std::map<RoadSegment*, Gate*> gts = isec->getOutGates();
        std::map<Gate*, double> gate2Dist;

        for (auto entry = gts.begin(); entry != gts.end(); entry++ ) {
            gate2Dist[entry->second] = (entry->second->getEndpoint()->getNode()->getPosition() - buildingPos).norm();
        }
        gatesDistance.insert(gate2Dist.begin(), gate2Dist.end());
    }

    // find out which gate is the nearest to the building
    auto nearest = std::min_element(gatesDistance.begin(), gatesDistance.end(),
                     [](std::pair<Gate* const, double>& left, std::pair<Gate* const, double>& right) -> bool {
                         return left.second < right.second;
                     });

    return nearest->first;
}

BuildingToGate nearestGateToBuilding(Buildings _buildings)
{
    BuildingToGate buildingToGate;

    std::map<Building*, Vector3d> buildingPositions;

    // Retrieve the positions of all the buildings
    for(Building* _b : _buildings) {
        buildingPositions[_b] = getBuildingPosition(_b);
    }

    std::map<Node*, Intersection*> isecs = World::getInstance()->getIntersections();
    std::map<Building*, std::pair<Gate*, double>> gatesDistanceToBuildings;

    // Compute the distance to all gates for each building
    // Save only the smallest one to each building
    for (const auto& it : isecs) {
        Intersection* isec = it.second;
        std::map<RoadSegment*, Gate*> gts = isec->getOutGates();

        for(Building* _b : _buildings) {
            for (auto entry = gts.begin(); entry != gts.end(); entry++ ) {
                double gateDist = (entry->second->getEndpoint()->getNode()->getPosition() -
                        buildingPositions[_b]).norm();
                if (
                        !gatesDistanceToBuildings.count(_b) ||
                        gateDist < gatesDistanceToBuildings[_b].second
                    ) {
                    gatesDistanceToBuildings[_b] = std::make_pair(entry->second, gateDist);
                }

            }
        }
    }

    // Retrieve only the building to nearest gate mapping
    for (auto entry = gatesDistanceToBuildings.begin();
         entry != gatesDistanceToBuildings.end();
         entry++) {
        buildingToGate[entry->first] = entry->second.first;
    }

    return buildingToGate;
}

BuildingToEndpoint nearestEndpointToBuilding(Buildings _buildings)
{
    BuildingToEndpoint buildingToEndpoint;

    std::map<Building*, Vector3d> buildingPositions;

    // Retrieve the positions of all the buildings
    for(Building* _b : _buildings) {
        buildingPositions[_b] = getBuildingPosition(_b);
    }

    std::map<std::string, Road*> roads = World::getInstance()->getRoads();
    std::map<Building*, std::pair<Endpoint*, double>> nodeDistanceToBuildings;

    // Compute the distance to all segment endpoints for each building
    // Save only the endpoint with the smallest distance to each building
    for(Building* _b : _buildings) {
        for (const auto & r_entry : roads) {
            Road* road = r_entry.second;
            std::vector<RoadSegment*> segments = road->getSegments();
            for ( auto segment : segments) {
                double dist1 = (
                            segment->getFromEndpoint()->getNode()->getPosition() -
                            buildingPositions[_b]
                            ).norm();
                double dist2 = (
                            segment->getToEndpoint()->getNode()->getPosition() -
                            buildingPositions[_b]
                            ).norm();

                if (dist1 < dist2 && (
                        !nodeDistanceToBuildings.count(_b) ||
                        dist1 < nodeDistanceToBuildings[_b].second)
                ) {
                    nodeDistanceToBuildings[_b] = std::make_pair(
                                segment->getFromEndpoint(),
                                dist1);

                } else if (dist2 < dist1 && (
                               !nodeDistanceToBuildings.count(_b) ||
                               dist2 < nodeDistanceToBuildings[_b].second
                    )
                ){
                    nodeDistanceToBuildings[_b] = std::make_pair(
                                segment->getToEndpoint(),
                                dist2);
                }
            }
        }
    }

    // Retrieve only the building to nearest endpoint mapping
    for (auto entry = nodeDistanceToBuildings.begin();
         entry != nodeDistanceToBuildings.end();
         entry++) {
        buildingToEndpoint[entry->first] = entry->second.first;
    }

    return buildingToEndpoint;
}

Endpoint* nearestEndpointToBuilding(Building *_building)
{
    Vector3d buildingPosition = getBuildingPosition(_building);
    std::map<std::string, Road*> roads = World::getInstance()->getRoads();
    std::map<Endpoint*, double> endpointsDistances;
    Endpoint * nearest = nullptr;
    double distance = std::numeric_limits<double>::infinity();

    for (const auto& r_entry : roads) {
        Road* road = r_entry.second;
        std::vector<RoadSegment*> segments = road->getSegments();

        for ( auto segment : segments) {
            double dist1 = (
                        segment->getFromEndpoint()->getNode()->getPosition() - buildingPosition
                        ).norm();
            double dist2 = (
                        segment->getToEndpoint()->getNode()->getPosition() - buildingPosition
                        ).norm();

            if (dist1 < distance && dist1 <= dist2) {
                distance = dist1;
                nearest = segment->getFromEndpoint();
            } else if (dist2 < distance){
                distance = dist2;
                nearest = segment->getToEndpoint();
            }
        }
    }
    return nearest;
}

RandomBuildingSelection selectRandomBuildings(uint _buildingCount, StringVector _excluded) {
    auto buildings = World::getInstance()->getBuildings();
    size_t buildingSize = buildings.size();

    StringVector buildingIds;
    std::vector<Endpoint*> endpoints;
    uint i = 0;
    const uint max = static_cast<uint>(buildingSize)-1, min = 0;
    while ( i < _buildingCount &&  buildingSize > 0) {
        auto it = buildings.begin();
        const uint randomIndex = (static_cast<uint>(rand()) % (max + 1 - min)) + min;
        std::advance(it, randomIndex);
        std::string id = it->first;

        Endpoint* endpoint = world::utils::nearestEndpointToBuilding(buildings.at(id));

        if (
                !LIMoSim::utils::vector::contains(endpoints, endpoint) && // prevent buildings having same endpoints
                !LIMoSim::utils::vector::contains(_excluded, id) && // exclude unreachable buildings
                std::find(buildingIds.begin(), buildingIds.end(), id) == buildingIds.end() // avoid duplicate buildings
                ) {
            buildingIds.push_back(id);
            endpoints.push_back(endpoint);
            i++;
            buildingSize--;
        }
    }
    RandomBuildingSelection selection;
    selection.buildingIds = buildingIds;
    selection.buildingEndpoints = endpoints;
    return selection;
}

bool isIntersecting(const Vector3d &_p1, const Vector3d &_p2, const Vector3d &_q1, const Vector3d &_q2)
{
    return (((_q1.x-_p1.x)*(_p2.y-_p1.y) - (_q1.y-_p1.y)*(_p2.x-_p1.x))
                * ((_q2.x-_p1.x)*(_p2.y-_p1.y) - (_q2.y-_p1.y)*(_p2.x-_p1.x)) < 0)
                &&
               (((_p1.x-_q1.x)*(_q2.y-_q1.y) - (_p1.y-_q1.y)*(_q2.x-_q1.x))
                * ((_p2.x-_q1.x)*(_q2.y-_q1.y) - (_p2.y-_q1.y)*(_q2.x-_q1.x)) < 0);
}


}
}
}
