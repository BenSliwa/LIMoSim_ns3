#include "lanesegment.h"
#include "roadsegment.h"
#include "LIMoSim/mobility/car/car.h"

namespace LIMoSim
{

LaneSegment::LaneSegment(RoadSegment *_roadSegment, int _type, int _index) :
    WorldObject("LaneSegment"),
    p_roadSegment(_roadSegment),
    m_type(_type),
    m_index(_index),
    p_next(0)
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

double LaneSegment::computeLength()
{
    return (m_end-m_start).norm();
}

Vector3d LaneSegment::computeDirection()
{
    return (m_end-m_start);
}

Node* LaneSegment::getStartNode()
{
    if(m_type==LANE_TYPE::FORWARD)
        return p_roadSegment->getFromEndpoint()->getNode();
    else
        return p_roadSegment->getToEndpoint()->getNode();
}

Node* LaneSegment::getEndNode()
{
    if(m_type==LANE_TYPE::FORWARD)
        return p_roadSegment->getToEndpoint()->getNode();
    else
        return p_roadSegment->getFromEndpoint()->getNode();
}

void LaneSegment::deregisterVehicle(Car *_car)
{
    if(m_cars.count(_car))
        m_cars.erase(_car);
}

void LaneSegment::updateVehicle(Car *_car)
{
    if(!m_cars.count(_car))
        m_cars[_car] = 0;
    m_cars[_car] = _car->getRoadPosition().offset_m;
}

Car* LaneSegment::findLeaderVehicle(double _offset_m, double _awareness_m, const std::string &_id)
{
    Car *leader = 0;

    std::map<Car*, double>::iterator it;
    double minDistance_m = computeLength();
    for(it=m_cars.begin(); it!=m_cars.end(); it++)
    {
        Car *car = it->first;
        double distance_m = it->second;

        bool c0 = distance_m-_offset_m<minDistance_m;
        bool c1 = distance_m>=_offset_m;
        bool c2 = distance_m<_awareness_m;

        if(car->getId()!=_id && distance_m-_offset_m<minDistance_m && distance_m>=_offset_m && distance_m<_awareness_m)
        {
            minDistance_m = distance_m - _offset_m;
            leader = car;
        }
    }

    return leader;
}

std::map<Car*, double> LaneSegment::getVehicles()
{
    return m_cars;
}

void LaneSegment::setNext(LaneSegment *_next)
{
    p_next = _next;
}

void LaneSegment::setStart(const Vector3d &_position)
{
    m_start = _position;
}

void LaneSegment::setEnd(const Vector3d &_position)
{
    m_end = _position;
}

int LaneSegment::getType()
{
    return m_type;
}

int LaneSegment::getIndex()
{
    return m_index;
}

LaneSegment* LaneSegment::getNext()
{
    return p_next;
}

Vector3d LaneSegment::getStart()
{
    return m_start;
}


Vector3d LaneSegment::getEnd()
{
    return m_end;
}

RoadSegment* LaneSegment::getSegment()
{
    return p_roadSegment;
}



}
