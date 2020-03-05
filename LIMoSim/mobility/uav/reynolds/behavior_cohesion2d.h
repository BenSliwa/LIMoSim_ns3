#ifndef BEHAVIOR_COHESION2D_H
#define BEHAVIOR_COHESION2D_H

#include "behavior_cohesion.h"

namespace LIMoSim {

class Behavior_Cohesion2D : public Behavior_Cohesion
{
public:
    Behavior_Cohesion2D(double _searchRadius = 200.0, std::string _agentId="");

    // Behavior interface
    Steering apply();

protected:
    Behavior_Cohesion2D( std::string _name, double _searchRadius = 200.0, std::string _agentId="");
};

} // namespace LIMoSim

#endif // BEHAVIOR_COHESION2D_H
