#ifndef BEHAVIOR_ARRIVE_H
#define BEHAVIOR_ARRIVE_H

#include "LIMoSim/mobility/uav/reynolds/behavior.h"
#include "LIMoSim/mobility/uav/reynolds/target.h"

namespace LIMoSim {

class UAV;

class Behavior_Arrive : public Behavior
{
public:
    Behavior_Arrive(Target _target, double arrivalRadius, std::string _agentId = "");
    Behavior_Arrive(Target _target, std::string _agentId = "");

private:
    Target m_target;
    double m_arrivalRadius;

    // Behavior interface
public:
    virtual Steering apply();
};

} // namespace LIMoSim

#endif // BEHAVIOR_ARRIVE_H
