#include "intersectionhandler.h"
#include "LIMoSim/world/road/roaddefinitions.h"
#include "LIMoSim/world/road/intersection.h"
#include "LIMoSim/world/road/roadsegment.h"
#include "LIMoSim/world/road/road.h"
#include "LIMoSim/mobility/car/car.h"

namespace LIMoSim
{

IntersectionHandler::IntersectionHandler()
{

}


/*************************************
 *            PUBLIC METHODS         *
 ************************************/

int IntersectionHandler::getTurnType(RoadSegment *_from, RoadSegment *_to, Intersection *_via)
{
    Node *via = _via->getNode();
    Node *to = _to->getOtherNode(via);
    Node *from = _from->getOtherNode(via);

    double phi0 = (from->getPosition() - via->getPosition()).computePhi();
    double phi1 = (to->getPosition() - via->getPosition()).computePhi();
    double angle = Vector3d::computeAngleDifference(phi0, phi1);

    return _via->getTurnType(angle);
}

std::string IntersectionHandler::getTurnTypeDescriptor(int _turnType)
{
    std::map<int, std::string> types;
    types[TURN_TYPE::STRAIGHT] = "STRAIGHT";
    types[TURN_TYPE::SLIGHT_LEFT] = "SLIGHT_LEFT";
    types[TURN_TYPE::LEFT] = "LEFT";
    types[TURN_TYPE::SHARP_LEFT] = "SHARP_LEFT";
    types[TURN_TYPE::SLIGHT_RIGHT] = "SLIGHT_RIGHT";
    types[TURN_TYPE::RIGHT] = "RIGHT";
    types[TURN_TYPE::SHARP_RIGHT] = "SHARP_RIGHT";

    return types[_turnType];
}

bool IntersectionHandler::drive(Car *_car, double _distance_m, RoadSegment *_segment, int _turnIntent, Intersection *_intersection)
{
    bool drive = true;

    // get all lanes that have to be considered
    std::map<Car*, CarEntry> approachingCars = _intersection->getApproachingCars();


    // find the nearest vehicle                                                             // TODO: we should switch to headway here
    double minDistance_m = 1000;
    std::map<Car*, CarEntry>::iterator it;
    std::vector<Car*> cars;
    for(it=approachingCars.begin(); it!=approachingCars.end(); it++)
    {
        CarEntry entry = it->second;
        if(it->first!=_car && _segment!=entry.segment)
        {
            Car *otherVehicle = it->first;
            RoadPosition info = otherVehicle->getRoadPosition();

            // here we only consider the nearest vehicles for each lane
            RoadAwarenessEntry leaderEntry = otherVehicle->getLeader();
            if(leaderEntry.car && leaderEntry.distance_m < info.distanceToIntersection_m)
                continue;

            // this is a hacky way of detecting outliers which might cause a deadlock
            if((otherVehicle->getPosition()-_intersection->getNode()->getPosition()).norm()>150)
            {
                continue;
            }

            int turnType = getTurnType(_segment, entry.segment, _intersection);
            bool consider = considerSegment(_segment, entry.segment, turnType, _turnIntent, info.turnIntent);

            double distance_m = _distance_m + entry.distance_m;
            if(consider)
            {
                cars.push_back(it->first);
                if(distance_m<minDistance_m)
                    minDistance_m = distance_m;
            }
        }
    }


    if(minDistance_m<50)
    {
        drive = false;

        bool deadlock = true;
        bool token = true;
        for(auto car : cars)
        {
            if(car->getMovementState()!=MOVEMENT_STATE::WAITING_VEHICLE)
                deadlock = false;
            if(car<_car)
                token = false;
        }

        if(deadlock && token)
        {
            drive = true;
        //    std::cout << _car->getId() << "\tusing token for deadlock compensation" << std::endl;
        }

        if(!drive && false)
        {
            std::cout << _car->getId() << "\twaiting\t" << cars.size();
            for(auto car:cars)
                std::cout << "\t" << car->getId() << " : " << car->getMovementState() << " : " << car->getRoadPosition().distanceToIntersection_m;
            std::cout << std::endl;
        }

    }



    // stop if the minimum headway cannot be guaranteed



    return drive;
}

/*************************************
 *           PRIVATE METHODS         *
 ************************************/

bool IntersectionHandler::considerSegment(RoadSegment *_from, RoadSegment *_to, int _turnType, int _turnIntent, int _intentOtherVehicle)
{
    bool consider = false;

    int intent = getTurnDirectionCategory(_turnIntent);
    int turn = getTurnDirectionCategory(_turnType);
    int transition = getTransitionCategory(_from->getRoad()->getType(), _to->getRoad()->getType());

    // TODO: here we shouldn't consider from->to transition, but rather from->segment hierarchy




    if(transition==TRANSITION_TYPE::SUPERIOR)
    {
        if(intent==TURN_TYPE::LEFT && turn==TURN_TYPE::STRAIGHT)
            consider = true;
    }
    else if(transition==TRANSITION_TYPE::INFERIOR)
    {
        if(intent==TURN_TYPE::LEFT) // all
            consider = true;
        else if(intent==TURN_TYPE::STRAIGHT && turn!=TURN_TYPE::STRAIGHT) // left & right
            consider = true;
        else if(intent==TURN_TYPE::RIGHT && turn==TURN_TYPE::LEFT) // left
            consider = true;
    }
    else if(transition==TRANSITION_TYPE::EQUAL) // rechts vor links
    {
        if(intent==TURN_TYPE::LEFT && turn!=TURN_TYPE::LEFT) // straight & right
        {
            if(turn==TURN_TYPE::STRAIGHT && _intentOtherVehicle==TURN_TYPE::LEFT)
                consider = false;
            else
                consider = true;
        }
        else if(intent==TURN_TYPE::STRAIGHT && turn==TURN_TYPE::RIGHT) // right
            consider = true;
    }

    return consider;
}

int IntersectionHandler::getTransitionCategory(int _fromWayType, int _toWayType)
{
    // TODO: integrate support for more way categories

    /*
    if(_fromWayType<_toWayType)
        return TRANSITION_TYPE::SUPERIOR;
    else if(_fromWayType>_toWayType)
        return TRANSITION_TYPE::INFERIOR;
    else
        return TRANSITION_TYPE::EQUAL;*/


    if(_fromWayType==ROAD_TYPE::PRIMARY && _toWayType==ROAD_TYPE::RESIDENTIAL)
        return TRANSITION_TYPE::SUPERIOR;
    else if(_fromWayType==ROAD_TYPE::RESIDENTIAL && _toWayType==ROAD_TYPE::PRIMARY)
        return TRANSITION_TYPE::INFERIOR;
    return TRANSITION_TYPE::EQUAL;

}

int IntersectionHandler::getTurnDirectionCategory(int _turnType)
{
    int t = _turnType;
    if(t==TURN_TYPE::STRAIGHT)
        return TURN_TYPE::STRAIGHT;
    else if(t==TURN_TYPE::SLIGHT_LEFT || t==TURN_TYPE::LEFT || t==TURN_TYPE::SHARP_LEFT)
        return TURN_TYPE::LEFT;
    else if(t==TURN_TYPE::SLIGHT_RIGHT|| t==TURN_TYPE::RIGHT || t==TURN_TYPE::SHARP_RIGHT)
        return TURN_TYPE::RIGHT;
}

}
