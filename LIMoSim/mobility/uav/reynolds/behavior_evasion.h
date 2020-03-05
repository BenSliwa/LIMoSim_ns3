#ifndef BEHAVIOR_EVASION_H
#define BEHAVIOR_EVASION_H

#include "LIMoSim/mobility/uav/reynolds/behavior.h"
#include "LIMoSim/mobility/uav/reynolds/target.h"

namespace LIMoSim {

class Behavior_Evasion : public Behavior
{
public:
    Behavior_Evasion(Target _target, std::string _agentId="");

    // Action interface
    Steering apply();


protected:
    Vector3d predictTargetPosition();

private:
    Target m_target;
};

} // namespace LIMoSim

#endif // BEHAVIOR_EVASION_H
