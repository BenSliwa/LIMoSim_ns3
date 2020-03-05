#ifndef ACTION_CIRCLEAROUND_H
#define ACTION_CIRCLEAROUND_H

#include "LIMoSim/world/vector3d.h"
#include "LIMoSim/mobility/uav/reynolds/behavior.h"
#include "LIMoSim/mobility/uav/reynolds/target.h"

namespace LIMoSim {
class UAV;

class Behavior_CircleAround: public Behavior
{
public:
    Behavior_CircleAround(Target _center, double _distance, std::string _agentId="");

    static std::string name();

private:
    Target m_center;
    double m_distance;

    // Action interface
public:
    Steering apply();
};


}

#endif // ACTION_CIRCLEAROUND_H
