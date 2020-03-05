#include "behavior_composition.h"

namespace LIMoSim {

Behavior_Composition::Behavior_Composition(std::string _name, Behaviors _behaviors, CompositionMode _compositionMode, std::string _agentId):
    Behavior(_name, _agentId),
    m_compositioMode(_compositionMode),
    m_behaviors(_behaviors)
{

}

Behavior_Composition::~Behavior_Composition()
{
    for (auto b : m_behaviors) {
        delete b;
        b = nullptr;
    }
}

Steering Behavior_Composition::apply()
{
    switch(m_compositioMode) {
    case PRIORITY_BY_ORDER:
        return applyWithPriorityByOrder();
    case SUPERPOSITION:
        return applyWithSuperposition();
    default:
        return applyWithPriorityByOrder();
    }
}

CompositionMode Behavior_Composition::getCompositionMode()
{
    return m_compositioMode;
}

void Behavior_Composition::setAgent(std::string _agentId)
{
    Behavior::setAgent(_agentId);

    for (auto b : m_behaviors) {
        b->setAgent(_agentId);
    }
}

void Behavior_Composition::setCompositionMode(CompositionMode _mode)
{
    m_compositioMode = _mode;
}

Steering Behavior_Composition::applyWithPriorityByOrder()
{
    Steering result;
    for (Behavior *b : m_behaviors) {
        result = b->apply();
        if (result.orientation.norm() || result.position.norm()) {
            return result;
        } else {
            continue;
        }
    }

    // no behavior delivered a non null steering
    // return a null steering anyway
    return result;

}

Steering Behavior_Composition::applyWithSuperposition()
{
    return applyNormalizedBehaviors(m_behaviors, false);
}

} // namespace LIMoSim
