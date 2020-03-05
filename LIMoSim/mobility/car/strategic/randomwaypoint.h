#ifndef LIMOSIM_RANDOMWAYPOINT_H
#define LIMOSIM_RANDOMWAYPOINT_H

#include "followpath.h"

namespace LIMoSim
{

class RandomWaypoint : public FollowPath
{
public:
    RandomWaypoint(Car *_car);

    void initialize();
    void handleNodeReached(Node *_node);
    void handleGateReached(Gate *_gate, Intersection *_intersection, LaneSegment *_lane);

private:
    Gate* computeRandomDestination();

};


}

#endif // LIMOSIM_RANDOMWAYPOINT_H
