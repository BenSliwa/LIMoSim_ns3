#ifndef BEHAVIOR_SELFALIGNMENT_H
#define BEHAVIOR_SELFALIGNMENT_H

#include "LIMoSim/mobility/uav/reynolds/behavior.h"

namespace LIMoSim {

class Behavior_SelfAlignment : public Behavior
{
public:
    Behavior_SelfAlignment(std::string _agentId = "");

    /**
     * @brief composeWith
     * Wrap a specific behavior in a superposed behavior composition
     * with self alignment.
     * @param _behavior
     * @param _agent
     * @return
     */
    static Behavior* composeWith(Behavior* _behavior, std::string _agentId="");

    // Behavior Interface
    Steering apply();
    Orientation3d alignOnVelocity(Vector3d _velocity);

};

} // namespace LIMoSim

#endif // BEHAVIOR_SELFALIGNMENT_H
