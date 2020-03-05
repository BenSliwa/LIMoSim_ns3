#ifndef BEHAVIOR_SEPARATION_H
#define BEHAVIOR_SEPARATION_H

#include <map>
#include "LIMoSim/mobility/uav/reynolds/behavior.h"

namespace LIMoSim {

class UAV;
class Vehicle;
class MobilityData;

class Behavior_Separation : public Behavior
{
public:
    Behavior_Separation(double _searchRadius = 100.0, std::string _agent = "");

    // Action interface
    Steering apply();

protected:
    std::map<std::string, MobilityData> findNearByUAVs();

private:
    double m_searchRadius;
};

} // namespace LIMoSim

#endif // BEHAVIOR_SEPARATION_H
