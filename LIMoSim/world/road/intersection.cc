#include "intersection.h"
#include "road.h"
#include "LIMoSim/world/world.h"
#include "LIMoSim/mobility/car/car.h"
#include <math.h>

namespace LIMoSim
{

Intersection::Intersection(Node *_node) :
    WorldObject("RoadIntersection"),
    p_node(_node)
{
    p_world = World::getInstance();
}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void Intersection::initialize(const std::vector<Road*> &_roads)
{
    // TODO: clear gates

    for(unsigned int i=0; i<_roads.size(); i++) // for each road
    {
        std::vector<RoadSegment*> segments = _roads.at(i)->getSegments();
        for(unsigned int j=0; j<segments.size(); j++) // find all segments that contain the node and the respective connection type
        {
            RoadSegment *segment = segments.at(j);
            if(segment->getFromEndpoint()->getNode()==p_node) // outgoing
            {
                if(segment->getRoad()->getForwardLanes()>0)
                {
                    Gate *out = new Gate(segment->getFromEndpoint());
                    m_outGates[segment] = out;
                }

                Gate *in = new Gate(segment->getFromEndpoint());
                m_inGates[segment] = in;

            }
            else if(segment->getToEndpoint()->getNode()==p_node) // incoming
            {
                if(segment->getRoad()->getBackwardLanes()>0)
                {
                    Gate *out = new Gate(segment->getToEndpoint());
                    m_outGates[segment] = out;
                }

                Gate *in = new Gate(segment->getToEndpoint());
                m_inGates[segment] = in;
            }

            // TODO: end nodes are in and out
        }
    }


    // TODO: handle loopback
}

void Intersection::exploreDestinations()
{
    // move forward on all outgoing gates until an intersection is found
    std::map<RoadSegment*, Gate*>::iterator it;
    for(it=m_outGates.begin(); it!=m_outGates.end(); it++)
    {
        Gate *outGate = it->second;
        GateConnection connection = exploreDestination(outGate);

        RoadSegment *origin = connection.out->getEndpoint()->getOwner();
        RoadSegment *seg = connection.in->getEndpoint()->getOwner();

        // update the routing capabilities of the gate
        connection.destination->updateConnections(outGate, connection.in, connection.distance_m);

        //std::cout << "CON: N" << outGate->toString() << " to N" << connection.destination->toString() << ": " << connection.distance_m << " m" << std::endl;

        //std::cout << outGate->toString() << "_out: found destination " << connection.in->toString() << "_in d=" << connection.distance_m;
        //std::cout << "\t W" << seg->getRoad()->getId() << "_" << seg->getIndex() << "_in";
        //std::cout << std::endl;
    }
}

GateConnection Intersection::exploreDestination(Gate *_gate)
{
    Intersection *intersection = 0;
    RoadSegment *segment = _gate->getEndpoint()->getOwner();
    Road *road = segment->getRoad();

    GateConnection connection;
    connection.out = _gate;
    connection.distance_m = segment->getLength();

    // determine the exploration direction
    int inc = 0;
    if(segment->getFromEndpoint()->getNode()==_gate->getEndpoint()->getNode())
        inc = 1; // increasing
    else
        inc = -1; // decreasing

    // move until an intersection is found
    Node *node = segment->getOtherNode(_gate->getEndpoint()->getNode());
    intersection = p_world->getIntersection(node);
    while(!intersection)
    {
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

        connection.distance_m += segment->getLength();
        node = segment->getOtherNode(node);
        intersection = p_world->getIntersection(node);
    }

    connection.in = intersection->getInGate(segment);
    connection.destination = intersection;

    return connection;
}

Road* Intersection::getNextExplorationRoad(Road *_road, Node *_node)
{
    // as the node is no intersection, it has exactly two roads
    std::vector<Road*> roads = p_world->filterRoads(_node->getWays());
    if(roads.at(0)==_road)
        return roads.at(1);
    else if(roads.at(1)==_road)
        return roads.at(0);
    return 0;
}

Gate* Intersection::getInGate(RoadSegment *_segment)
{
    if(m_inGates.count(_segment)>0)
        return m_inGates[_segment];
    return 0;
}

Gate* Intersection::getOutGate(RoadSegment *_segment)
{
    if(m_outGates.count(_segment)>0)
        return m_outGates[_segment];
    return 0;
}

RoadSegment* Intersection::getOutSegmentForGate(Gate *_gate)
{
    std::map<RoadSegment*, Gate*>::iterator it;
    for(it=m_outGates.begin(); it!=m_outGates.end(); it++)
    {
        if(it->second==_gate)
            return it->first;
    }
    return 0;
}

void Intersection::updateConnections(Gate *_out, Gate *_in, double _distance_m)
{
    // connect the out gate of the neighboring intersection to all reachable gates of this intersection


    // TODO: only include the out gates that are reachable from in
    std::map<RoadSegment*, Gate*>::iterator it;
    for(it=m_outGates.begin(); it!=m_outGates.end(); it++)
    {
        // TODO: add the distance required for passing the intersection
        Gate *gate = it->second;

        // avoid u-turns
        if(m_outGates.size()>1 && _in->getEndpoint()->getOwner()==gate->getEndpoint()->getOwner())
        {

        }
        else
        {
            _out->addNeighbor(gate, _distance_m);
        }
    }
}

int Intersection::getTurnType(double _angle)
{
    int turnType = TURN_TYPE::STRAIGHT;

    bool negative = _angle<0.0;
    double value = fabs(_angle);

    if(value>25 && value<50)
        negative ? turnType = TURN_TYPE::SLIGHT_LEFT : turnType = TURN_TYPE::SLIGHT_RIGHT;
    else if(value>=50 && value<100)
        negative ? turnType = TURN_TYPE::LEFT : turnType = TURN_TYPE::RIGHT;
    else if(value >=100 && value<150)
        negative ? turnType = TURN_TYPE::SHARP_LEFT : turnType = TURN_TYPE::SHARP_RIGHT;

    return turnType;
}

void Intersection::deregisterCar(Car *_car)
{
    if(m_approachingCars.count(_car))
        m_approachingCars.erase(_car);
}

void Intersection::updateCar(Car *_car, RoadSegment *_segment, double _distance_m)
{
    if(!m_approachingCars.count(_car))
        m_approachingCars[_car] = CarEntry();

    m_approachingCars[_car].segment = _segment;
    m_approachingCars[_car].distance_m = _distance_m;
    m_approachingCars[_car].headway_s = _distance_m / _car->getSpeed();
}

std::map<Car*, CarEntry> Intersection::getApproachingCars()
{
    return m_approachingCars;
}

CarEntry Intersection::getCarEntry(Car *_car)
{
    return m_approachingCars[_car];
}

Node* Intersection::getNode()
{
    return p_node;
}

const std::map<RoadSegment*, Gate*>& Intersection::getOutGates()
{
    return m_outGates;
}

const std::map<RoadSegment*, Gate*>& Intersection::getInGates()
{
    return m_inGates;
}

int Intersection::getOutSize()
{
    return m_outGates.size();
}

std::string Intersection::toString()
{
    return p_node->getId();
}

}
