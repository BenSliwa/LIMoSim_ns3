#include "behaviors.h"

namespace LIMoSim {
Behavior *makeSelfAligned(Behavior *_behavior, std::string _agentId) {
    return Behavior_SelfAlignment::composeWith(_behavior, _agentId);
}
}
