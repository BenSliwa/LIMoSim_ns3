#ifndef BEHAVIOR_NOOP_H
#define BEHAVIOR_NOOP_H

#include "behavior.h"

namespace LIMoSim {

class UAV;

class Behavior_NoOp : public Behavior
{
public:
    Behavior_NoOp();

    // Behavior interface
    Steering apply();
};

} // namespace LIMoSim

#endif // BEHAVIOR_NOOP_H
