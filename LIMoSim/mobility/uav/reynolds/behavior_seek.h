#ifndef BEHAVIOR_SEEK_H
#define BEHAVIOR_SEEK_H


#include "LIMoSim/mobility/uav/reynolds/behavior.h"
#include "LIMoSim/mobility/uav/reynolds/target.h"

namespace LIMoSim {

class UAV;

class Behavior_Seek : public Behavior
{
public:
    Behavior_Seek(Target _target, std::string _agentId="");

    // Behavior interface
    Steering apply();

protected:
    Target m_target;
};

} // namespace LIMoSim

#endif // BEHAVIOR_SEEK_H
