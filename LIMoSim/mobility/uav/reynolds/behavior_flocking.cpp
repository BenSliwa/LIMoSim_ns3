#include "behavior_flocking.h"
#include "behavior_cohesion.h"
#include "behavior_separation.h"

namespace LIMoSim {

Behavior_Flocking::Behavior_Flocking(std::string _agentId):
    Behavior("Flocking", _agentId)
{

}

Behavior_Flocking::~Behavior_Flocking()
{
    for (Behavior* b : m_behaviors) {
        delete b;
        b = nullptr;
    }
}

Steering Behavior_Flocking::apply()
{
    Steering steering = applyNormalizedWeightedBehaviors(m_behaviors, Weights({1,1}), false);
    return steering;
}

void Behavior_Flocking::setAgent(std::string _agentId)
{
    Behavior::setAgent(_agentId);

    if (!m_behaviors.empty()) {
        for (Behavior* b : m_behaviors) {
            delete b;
            b = nullptr;
        }
    }

    m_behaviors = {
        new Behavior_Cohesion(200, m_agentId),
        new Behavior_Separation(50, m_agentId)
    };
}

} // namespace LIMoSim
