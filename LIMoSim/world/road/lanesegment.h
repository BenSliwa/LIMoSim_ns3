#ifndef LIMOSIM_LANESEGMENT_H
#define LIMOSIM_LANESEGMENT_H

#include "LIMoSim/world/worldobject.h"
#include "LIMoSim/world/node.h"
#include <map>

namespace LIMoSim
{

class RoadSegment;
class Car;

namespace LANE_TYPE
{
    enum
    {
        FORWARD,
        BACKWARD
    };
}

class LaneSegment : public WorldObject
{
public:
    LaneSegment(RoadSegment *_roadSegment, int _type, int _index);

    double computeLength();
    Vector3d computeDirection();
    Node* getStartNode();
    Node* getEndNode();

    //
    void deregisterVehicle(Car *_car);
    void updateVehicle(Car *_car);
    Car* findLeaderVehicle(double _offset_m, double _awareness_m, const std::string &_id);
    std::map<Car*, double> getVehicles();


    //
    void setNext(LaneSegment *_next);
    void setStart(const Vector3d &_position);
    void setEnd(const Vector3d &_position);


    int getType();
    int getIndex();
    LaneSegment* getNext();
    Vector3d getStart();
    Vector3d getEnd();
    RoadSegment* getSegment();


private:
    RoadSegment *p_roadSegment;
    int m_type;
    int m_index;

    LaneSegment *p_next;

    Vector3d m_start;
    Vector3d m_end;

    std::map<Car*, double> m_cars;

};

}

#endif // LIMOSIM_LANESEGMENT_H
