#ifndef BEHAVIOR_IDSELECTIVECOHESION2D_H
#define BEHAVIOR_IDSELECTIVECOHESION2D_H

#include "behavior_cohesion2d.h"

namespace LIMoSim {

class UAV;
class Vehicle;
class MobilityData;

class Behavior_IdSelectiveCohesion2D: public Behavior_Cohesion2D
{
public:
    Behavior_IdSelectiveCohesion2D(std::vector<std::string> _relevantIds, double _searchRadius = 200.0, std::string _agentId="");

protected:
    virtual std::map<std::string, MobilityData> findNearByUAVs();
    std::vector<std::string> m_relevantIds;
};

} // namespace LIMoSim

#endif // BEHAVIOR_IDSELECTIVECOHESION2D_H
