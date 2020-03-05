#include "behavior_circlearound.h"
#include "behaviors.h"
#include "LIMoSim/mobility/uav/uav.h"


namespace  LIMoSim {

Behavior_CircleAround::Behavior_CircleAround(Target _center, double _distance, std::string _agentId):
    Behavior(name(), _agentId),
    m_center(_center),
    m_distance(_distance)
{

}

std::string Behavior_CircleAround::name()
{
    return "CircleAround";
}



Steering Behavior_CircleAround::apply()
{
    Behaviors behaviors;

    Vector3d centerToAgent = getAgent()->getPosition() - m_center.getPosition();

    Vector3d agentProjectionFromCenter;
    if (!centerToAgent.norm()) {
        agentProjectionFromCenter = centerToAgent + Vector3d(0, m_distance, 0);
    } else{
        agentProjectionFromCenter = centerToAgent.normed() * m_distance;
    }

    if (centerToAgent.norm() - m_distance > 10){
        // TODO: use arrival behavior to get on radius instead
        Vector3d agentProjection = agentProjectionFromCenter + m_center.getPosition();
        behaviors.push_back(makeSelfAligned(new Behavior_Seek(Target(agentProjection), m_agentId), m_agentId));
    } else {
        Vector3d rotatedAgentProjection = agentProjectionFromCenter.rotateTheta(0.1).normed() * m_distance + m_center.getPosition();
        behaviors.push_back(makeSelfAligned(new Behavior_Seek(Target(rotatedAgentProjection), m_agentId), m_agentId));
    }

    // TODO: add face center behavior

    return Behavior::applyBehaviors(behaviors);
}

}
