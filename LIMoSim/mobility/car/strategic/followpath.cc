#include "followpath.h"

namespace LIMoSim
{

FollowPath::FollowPath(Car *_car) :
    StrategicModel(_car)
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void FollowPath::handleNodeReached(Node *_node)
{

}

void FollowPath::handleGateReached(Gate *_gate, Intersection *_intersection, LaneSegment *_lane)
{
    RoadPosition info = p_car->getRoadPosition();

    std::cout << "FollowPath::handleGateReached " << _gate->toString() << " " << info.path.size() << std::endl;


    if(info.path.size()>0)
    {
        Gate *gate = info.path.at(0);
        RoadSegment *segment = _intersection->getOutSegmentForGate(gate);

        if(segment) // reached a gate of the path
        {
            //
            RoadSegment *segment = _intersection->getOutSegmentForGate(gate);
            info.path.erase(info.path.begin());
            info.laneSegment = getNextLaneSegment(segment, _intersection, _lane);
            info.offset_m = 0;
            p_car->setRoadPosition(info);
        }
        else // reached a gate that is not part of the path -> rerouting
        {
        }
    }
}


}
