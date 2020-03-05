#ifndef LIMOSIM_RANDOMDIRECTION_H
#define LIMOSIM_RANDOMDIRECTION_H

#include "strategicmodel.h"

namespace LIMoSim
{

class RandomDirection : public StrategicModel
{
public:
    RandomDirection(Car *_car);

    void initialize();
    void handleNodeReached(Node *_node);
    void handleGateReached(Gate *_gate, Intersection *_intersection, LaneSegment *_lane);
};

}


#endif // LIMOSIM_RANDOMDIRECTION_H
