#include "reynoldsmodel.h"
#include "behavior_noop.h"
#include "LIMoSim/mobility/uav/reynolds/behavior.h"
#include "LIMoSim/mobility/uav/uav.h"

namespace LIMoSim {

ReynoldsModel::ReynoldsModel(std::string _agentId,
                             Behavior *_behavior,
                             Locomotion *_locomotion):
    MobilityModel(_agentId),
    m_behavior(_behavior),
    m_locomotion(_locomotion)
{
    if (!m_behavior) {
        m_behavior = new Behavior_NoOp();
    }
}

ReynoldsModel::~ReynoldsModel()
{
    if (m_behavior) delete m_behavior;
    if (m_locomotion) delete m_locomotion;
}

UAV *ReynoldsModel::getAgent()
{
    return dynamic_cast<UAV*>(MobilityModel::getAgent());
}

Locomotion *ReynoldsModel::getLocomotion()
{
    return m_locomotion;
}

Behavior *ReynoldsModel::getBehavior()
{
    return m_behavior;
}

std::string ReynoldsModel::getBehaviorName()
{
    return m_behavior->getName();
}

void ReynoldsModel::setBehavior(Behavior *_behavior)
{
    if (m_behavior) delete m_behavior;
    m_behavior = _behavior;
}

LocomotionUpdate ReynoldsModel::step(double _timeDelta_s)
{
    return applyLocomotion(
                applyBehavior(m_behavior),
                _timeDelta_s
                );
}

Steering ReynoldsModel::applyBehavior(Behavior *_behavior)
{
    return _behavior ?_behavior->apply() : Steering();
}

LocomotionUpdate ReynoldsModel::applyLocomotion(Steering _steering, double _timeDelta_s)
{
    return m_locomotion->apply(_steering, _timeDelta_s);
}

}
