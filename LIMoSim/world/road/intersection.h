#ifndef LIMOSIM_INTERSECTION_H
#define LIMOSIM_INTERSECTION_H

#include "LIMoSim/world/worldobject.h"
#include "LIMoSim/world/node.h"
#include "gate.h"
#include <map>

/* Intersections are defined by the number of endpoints that refer to a node
 * N=1 or N>2: intersection
 * Intersections own and maintain Gates
 */

namespace LIMoSim
{

class Car;
class Intersection;
class Road;
class World;

struct GateConnection
{
    Gate *out = 0;
    Gate *in = 0;
    Intersection *destination = 0;
    double distance_m = 0;
};

struct CarEntry
{
    RoadSegment *segment = 0;
    double distance_m = 0;
    double headway_s = 0;
};

namespace TURN_TYPE
{
    enum{
        NONE,
        STRAIGHT,
        SLIGHT_LEFT,
        LEFT,
        SHARP_LEFT,
        SLIGHT_RIGHT,
        RIGHT,
        SHARP_RIGHT,
        U_TURN
    };
}

class Intersection : public WorldObject
{
public:
    Intersection(Node *_node);

    void initialize(const std::vector<Road*> &_roads);

    //
    void exploreDestinations();
    GateConnection exploreDestination(Gate *_gate);
    Road* getNextExplorationRoad(Road *_road, Node *_node);
    Gate* getInGate(RoadSegment *_segment);
    Gate* getOutGate(RoadSegment *_segment);

    //
    RoadSegment* getOutSegmentForGate(Gate *_gate);
    void updateConnections(Gate *_out, Gate *_in, double _distance_m);


    //
    int getTurnType(double _angle);

    //
    void deregisterCar(Car *_car);
    void updateCar(Car *_car, RoadSegment *_segment, double _distance_m);
    std::map<Car*, CarEntry> getApproachingCars();

    CarEntry getCarEntry(Car *_car);

    //
    Node* getNode();
    const std::map<RoadSegment*, Gate*>& getOutGates();
    const std::map<RoadSegment*, Gate*>& getInGates();
    int getOutSize();

    //
    std::string toString();

private:
    Node *p_node;
    World *p_world;

    std::map<RoadSegment*, Gate*> m_outGates; // out of the intersection, into the segment
    std::map<RoadSegment*, Gate*> m_inGates; // into the intersection, out of the segment

    std::map<Car*, CarEntry> m_approachingCars;
};

}

#endif // LIMOSIM_INTERSECTION_H
