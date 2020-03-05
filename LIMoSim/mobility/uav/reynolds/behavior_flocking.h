#ifndef BEHAVIOR_FLOCKING_H
#define BEHAVIOR_FLOCKING_H

#include "LIMoSim/mobility/uav/reynolds/behavior.h"

namespace LIMoSim {

class UAV;

class Behavior_Flocking : public Behavior
{
public:
    Behavior_Flocking(std::string _agentId="");
    ~Behavior_Flocking();

    // Action interface
    Steering apply();
    void setAgent(std::string _agentId);

private:
    Behaviors m_behaviors;
};

} // namespace LIMoSim

#endif // BEHAVIOR_FLOCKING_H
