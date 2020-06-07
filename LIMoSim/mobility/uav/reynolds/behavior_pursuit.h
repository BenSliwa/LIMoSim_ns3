#ifndef BEHAVIOR_PURSUIT_H
#define BEHAVIOR_PURSUIT_H

#include "LIMoSim/mobility/uav/reynolds/behavior.h"
#include "LIMoSim/mobility/uav/reynolds/target.h"

namespace LIMoSim {

class Behavior_Pursuit : public Behavior
{
public:
    Behavior_Pursuit(Target _target, std::string _agentId="");

    // Action interface
    Steering apply();

protected:
    Vector3d predictTargetPosition();

private:
    Target m_target;
};

} // namespace LIMoSim

#endif // BEHAVIOR_PURSUIT_H
