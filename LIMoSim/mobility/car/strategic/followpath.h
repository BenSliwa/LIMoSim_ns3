#ifndef LIMOSIM_FOLLOWPATH_H
#define LIMOSIM_FOLLOWPATH_H

#include "strategicmodel.h"

namespace LIMoSim
{

class FollowPath : public StrategicModel
{
public:
    FollowPath(Car *_car);

    void handleNodeReached(Node *_node);
    void handleGateReached(Gate *_gate, Intersection *_intersection, LaneSegment *_lane);

};

}

#endif // LIMOSIM_FOLLOWPATH_H
