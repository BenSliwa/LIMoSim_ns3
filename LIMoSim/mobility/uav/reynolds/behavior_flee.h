#ifndef BEHAVIOR_FLEE_H
#define BEHAVIOR_FLEE_H

#include "LIMoSim/mobility/uav/reynolds/behavior.h"
#include "LIMoSim/mobility/uav/reynolds/target.h"

namespace LIMoSim {

class UAV;

class Behavior_Flee : public Behavior
{
public:
    Behavior_Flee(Target _target, std::string _agentId="");

private:
    Target m_target;

    // Behavior interface
public:
    Steering apply();
};

} // namespace LIMoSim

#endif // BEHAVIOR_FLEE_H
