#include "randomdirection.h"


namespace LIMoSim
{

RandomDirection::RandomDirection(Car *_car) :
    StrategicModel(_car)
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void RandomDirection::initialize()
{
    RoadPosition info = p_car->getRoadPosition();
    info.path.clear();
    p_car->setRoadPosition(info);
}

void RandomDirection::handleNodeReached(Node *_node)
{

}

void RandomDirection::handleGateReached(Gate *_gate, Intersection *_intersection, LaneSegment *_lane)
{
    std::map<RoadSegment*, Gate*> out = _intersection->getOutGates();
    std::map<RoadSegment*, Gate*>::iterator it = out.begin();

    if(out.size()>0)
    {
        // find the destination gate
        RoadSegment* segment = it->first;
        if(out.size()>1)
        {
            while(it->first==_lane->getSegment())
            {
                int index = rand() * (long)(out.size()) / RAND_MAX;
                it = out.begin();
                for(int i=0; i<index; i++)
                    it++;

                segment = it->first;
            }
        }

        //
        RoadPosition info = p_car->getRoadPosition();
        info.laneSegment = getNextLaneSegment(segment, _intersection, _lane);
        info.offset_m = 0;
        p_car->setRoadPosition(info);
    }
}

}

