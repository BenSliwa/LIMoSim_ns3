#include "roadawareness.h"
#include "mobility/car/car.h"

namespace LIMoSim
{

RoadAwareness::RoadAwareness()
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

RoadAwarenessEntry RoadAwareness::findLeader(Car *_car)
{
    RoadAwarenessEntry entry;

    Car *leader = 0;
    double awarenessDistance_m = 100;

    // virtually travel forward -> TODO: consider the current trajectory
    LaneSegment *segment = _car->getRoadPosition().laneSegment;
    double offset_m = _car->getRoadPosition().offset_m;
    double distance_m = 0;
    while(!leader && distance_m<awarenessDistance_m)
    {
        leader = segment->findLeaderVehicle(offset_m, awarenessDistance_m-distance_m, _car->getId());

        //
        if(!leader)
        {
            // find the next segment if the awareness distance allows it
             LaneSegment *nextSegment = segment->getNext();
             if(!nextSegment)
             {
                // TODO: leader lookup with intersection

                //if(_car->getId()=="C0")
                //    std::cout << "no next segment" << std::endl;

                // terminate the lookup procedure
                distance_m = awarenessDistance_m;
             }
             else
             {
                 // update the virtually travelled distance
                 distance_m += segment->computeLength() - offset_m;
                 segment = nextSegment;
             }
        }

        if(leader)
        {
            distance_m += leader->getRoadPosition().offset_m;
            distance_m -= leader->getLength()/2.0;
            distance_m -= _car->getLength()/2.0;

            entry.car = leader;
            entry.speedDelta_mps = _car->getSpeed() - entry.car->getSpeed();
            entry.distance_m = distance_m - offset_m;
        }

        offset_m = 0;
    }

    return entry;
}

/*************************************
 *           PRIVATE METHODS         *
 ************************************/


}

