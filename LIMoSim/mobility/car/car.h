#ifndef LIMOSIM_CAR_H
#define LIMOSIM_CAR_H
#include <set>

#include "LIMoSim/mobility/vehicle.h"
#include "LIMoSim/world/road/lanesegment.h"
#include "LIMoSim/world/road/road.h"
#include "LIMoSim/mobility/car/awareness/roadawareness.h"
#include "LIMoSim/mobility/car/awareness/intersectionhandler.h"

namespace LIMoSim
{

class IDM;
class StrategicModel;
class Intersection;
class Node;
class Gate;
class World;

namespace MOVEMENT_STATE
{
    enum{
        DRIVING_FREE,
        DRIVING_FOLLOWING,
        WAITING_VEHICLE,
        WAITING_SIGNAL
    };
}

struct RoadPosition
{
    LaneSegment *laneSegment;
    double offset_m;
    std::vector<Gate*> path;

    //
    Intersection *intersection = 0;
    double distanceToIntersection_m;
    int turnIntent;
};

class Car : public Vehicle
{
public:
    Car(const std::string &_id);
    ~Car();

    //
    void setRoadPosition(const RoadPosition &_position);
    void setMovementState(int _state);
    void setSpeed(double _speed_mps);
    void setMaxSpeed(double _speed_mps);


    RoadPosition getRoadPosition();
    RoadAwarenessEntry getLeader();
    int getMovementState();
    double getWidth();
    double getLength();
    double getAcceleration();
    double getSpeed();
    double getMaxSpeed();
    StrategicModel* getStrategicModel();


public:
    void initialize();
    void handleEvent(Event *_event);
    void move(double _timeDelta_s);

private:
    void updateAcceleration();
    void updateTravelledDistance(double _timeDelta_s);
    void updatePosition();
    void updateIntersectionAwareness();
    double computeDesiredSpeed();
    void handleSegmentSwitch();


public:
    RoadAwareness m_awareness;

    RoadPosition m_roadPosition;
    RoadAwarenessEntry m_leader;

    int m_movementState;

    double m_width_m;
    double m_length_m;

    double m_acceleration_mpss;
    double m_speed_mps;
    double m_maxSpeed_mps;

    World *p_world;
    IDM *m_following;
    StrategicModel *m_strategic;
};

}

#endif // LIMOSIM_CAR_H
