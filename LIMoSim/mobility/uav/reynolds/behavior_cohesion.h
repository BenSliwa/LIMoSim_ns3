#ifndef BEHAVIOR_COHESION_H
#define BEHAVIOR_COHESION_H

#include <map>
#include "LIMoSim/mobility/uav/reynolds/behavior.h"

namespace LIMoSim {

class UAV;
class Vehicle;
class MobilityData;

class Behavior_Cohesion : public Behavior
{
public:
    Behavior_Cohesion(double _searchRadius = 200.0, std::string _agentId="");

    // Behavior interface
    Steering apply();

protected:
    Behavior_Cohesion( std::string _name, double _searchRadius = 200.0, std::string _agentId="");
    virtual std::map<std::string, MobilityData> findNearByUAVs();

    double m_searchRadius;
};

} // namespace LIMoSim

#endif // BEHAVIOR_COHESION_H
