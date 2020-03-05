#ifndef BEHAVIOR_WANDER_H
#define BEHAVIOR_WANDER_H

#include "behavior.h"

namespace LIMoSim {

class UAV;

class Behavior_Wander : public Behavior
{
public:
    Behavior_Wander(double _wanderingSpeed_mps = 10.0, std::string _agentId = "");

    // Action interface
    Steering apply();

private:
    double m_wanderingSpeed_mps;
    double m_wanderSphereDistance_m = 100;
    double m_wanderSphereRadius_m = 200;

    Vector3d wanderSphereCenter();
    Vector3d randomWanderSpherePoint();
};

} // namespace LIMoSim

#endif // BEHAVIOR_WANDER_H
