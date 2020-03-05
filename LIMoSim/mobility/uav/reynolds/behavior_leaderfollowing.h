#ifndef BEHAVIOR_LEADERFOLLOWING_H
#define BEHAVIOR_LEADERFOLLOWING_H

#include "LIMoSim/mobility/uav/reynolds/behavior.h"

namespace LIMoSim {

class MobilityData;

class Behavior_LeaderFollowing : public Behavior
{
public:
    Behavior_LeaderFollowing(std::string _leaderId, std::string _agentId="");


    UAV* getLeader();

    // Behavior interface
    Steering apply();

protected:
    Vector3d getFollowingTarget();
    Vector3d getLeadersDirection();
    bool inLeadersFrontSpace();
    MobilityData leadersMobilityData();

protected:
    std::string m_leaderId;
    double m_basicOffsetFromLeader_m;
    double m_leaderFrontspaceHeight_m;
    double m_leaderFrontspaceWidth_m;
};

} // namespace LIMoSim

#endif // BEHAVIOR_LEADERFOLLOWING_H
