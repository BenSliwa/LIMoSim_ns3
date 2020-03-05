#ifndef BEHAVIOR_COMPOSITION_H
#define BEHAVIOR_COMPOSITION_H

#include "LIMoSim/mobility/uav/reynolds/behavior.h"

namespace LIMoSim {

class UAV;

/**
 * @brief The CompositionMode enum
 * - PRIORITY_BY_ORDER: only applies the first behavior which results
 * in a non null steering;
 * - SUPERPOSITION: applies all behaviors and combine their locomotion-
 * normed steerings
 */
enum CompositionMode  { PRIORITY_BY_ORDER, SUPERPOSITION };

class Behavior_Composition : public Behavior
{
public:
    Behavior_Composition(std::string _name, Behaviors _behaviors, CompositionMode _compositionMode = PRIORITY_BY_ORDER, std::string _agentId="");
    ~Behavior_Composition();

    // Behavior interface
    Steering apply();

    //
    CompositionMode getCompositionMode();

    //
    void setAgent(std::string _agentId);
    void setCompositionMode(CompositionMode _mode);

protected:
    Steering applyWithPriorityByOrder();
    Steering applyWithSuperposition();

private:
    CompositionMode m_compositioMode;
    Behaviors m_behaviors;
};

} // namespace LIMoSim

#endif // BEHAVIOR_COMPOSITION_H
