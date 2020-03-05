#ifndef LIMOSIM_INTERSECTIONHANDLER_H
#define LIMOSIM_INTERSECTIONHANDLER_H

#include <string>

namespace LIMoSim
{

class Car;
class Intersection;
class RoadSegment;

class IntersectionHandler
{
public:
    IntersectionHandler();

    static int getTurnType(RoadSegment *_from, RoadSegment *_to, Intersection *_via);
    static std::string getTurnTypeDescriptor(int _turnType);

    bool drive(Car *_car, double _distance_m, RoadSegment *_segment, int _turnIntent, Intersection *_intersection);
    bool considerSegment(RoadSegment *_from, RoadSegment *_to, int _turnType, int _turnIntent, int _intentOtherVehicle);

private:
    int getTransitionCategory(int _fromWayType, int _toWayType);
    int getTurnDirectionCategory(int _turnType);

};

}
#endif // LIMOSIM_INTERSECTIONHANDLER_H
