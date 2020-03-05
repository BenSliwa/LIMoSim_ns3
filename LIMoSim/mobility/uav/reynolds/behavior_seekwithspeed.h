#ifndef BEHAVIOR_SEEKWITHSPEED_H
#define BEHAVIOR_SEEKWITHSPEED_H

#include "LIMoSim/mobility/uav/reynolds/behavior.h"
#include "LIMoSim/mobility/uav/reynolds/target.h"

namespace LIMoSim {

class Behavior_SeekWithSpeed : public Behavior
{
public:
    Behavior_SeekWithSpeed(Target _target, double _seekingSpeed, std::string _agentId="");

    // Behavior interface
    Steering apply();

protected:
    Target m_target;
    double m_seekingSpeed;
};

} // namespace LIMoSim

#endif // BEHAVIOR_SEEKWITHSPEED_H
