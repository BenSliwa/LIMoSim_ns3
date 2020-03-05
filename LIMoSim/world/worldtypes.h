#ifndef WORLDTYPES_H
#define WORLDTYPES_H


namespace LIMoSim {

class Intersection;
class RoadSegment;
class Routable;
class Road;
class Gate;

namespace world {
namespace types {

struct IntersectionDiscovery {
    Intersection *intersection = nullptr;
    /**
     * @brief segment
     * Road segment through which the intersection
     * should be left to reach the endpoint
     */
    RoadSegment *segment = nullptr;

    Gate *gate;
    double cost = 0;
};

} // namespace types
} // namespace world
} // namespace LIMoSim

#endif // WORLDTYPES_H
