#ifndef LIMOSIM_ROADAWARENESS_H
#define LIMOSIM_ROADAWARENESS_H

#include <vector>

namespace LIMoSim
{

class Car;
class LaneSegment;

struct RoadAwarenessEntry
{
    Car *car = 0;
    double distance_m = 0;
    double speedDelta_mps = 0;
};

class RoadAwareness
{
public:
    RoadAwareness();

    RoadAwarenessEntry findLeader(Car *_car);
};

}

#endif // LIMOSIM_ROADAWARENESS_H
