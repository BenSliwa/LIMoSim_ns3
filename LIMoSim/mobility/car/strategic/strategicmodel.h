#ifndef LIMOSIM_STRATEGICMODEL_H
#define LIMOSIM_STRATEGICMODEL_H

#include "LIMoSim/mobility/car/car.h"
#include "LIMoSim/world/road/gate.h"
#include "LIMoSim/world/road/intersection.h"
#include <iostream>

namespace LIMoSim
{

class StrategicModel
{
public:
    StrategicModel(Car *_car);
    virtual ~StrategicModel();

    virtual void initialize();
    virtual void handleNodeReached(Node *_node) = 0;
    virtual void handleGateReached(Gate *_gate, Intersection *_intersection, LaneSegment *_lane) = 0;

    LaneSegment* getNextLaneSegment(RoadSegment *_segment, Intersection *_intersection, LaneSegment *_lane);

protected:
    Car *p_car;
};

}


#endif // LIMOSIM_STRATEGICMODEL_H
