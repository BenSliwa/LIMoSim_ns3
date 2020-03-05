#include "behavior_leaderfollowing.h"

#include <algorithm>

#include "behaviors.h"

#include "LIMoSim/mobility/uav/uav.h"
#include "LIMoSim/world/vehiclemanager.h"

namespace LIMoSim {

Behavior_LeaderFollowing::Behavior_LeaderFollowing(std::string _leaderId, std::string _agentId):
    Behavior("Leader Following", _agentId),
    m_leaderId(_leaderId),
    m_basicOffsetFromLeader_m(20),
    m_leaderFrontspaceHeight_m(40),
    m_leaderFrontspaceWidth_m(20)
{

}

UAV *Behavior_LeaderFollowing::getLeader()
{
    return dynamic_cast<UAV*>(VehicleManager::getInstance()->getVehicle(m_leaderId));
}

Steering Behavior_LeaderFollowing::apply()
{
    auto it = getAgent()->getLocalVehicleManager()->getMobilityData().find(m_leaderId);

    // if no mobility data of leader available, do nothing
    if (it == getAgent()->getLocalVehicleManager()->getMobilityData().end()) {
        return Behavior_NoOp().apply();
    }

    MobilityData leaderMobility = leadersMobilityData();
    Vector3d currentPosition = getAgent()->getPosition();

    if (inLeadersFrontSpace()){
        // flee from leader's front space (controlled fleeing using seek)

        Vector3d distanceFromLeader = currentPosition - leaderMobility.position;
        Vector3d projectionOnLeaderTrajectory = leaderMobility.position +
                getLeadersDirection() *
                Vector3d::dot(distanceFromLeader, getLeadersDirection());
        Vector3d fleeDirection;

        // if we are on the leader's trajectory then evade right
        if (projectionOnLeaderTrajectory.norm() == currentPosition.norm()) {
            fleeDirection = distanceFromLeader.rotateLeft().normed();
        }
        // else evade on the side we are
        else {
            fleeDirection = (currentPosition - projectionOnLeaderTrajectory).normed();
        }

        return Behavior_Seek(currentPosition + fleeDirection * 20, getAgent()->getId()).apply();

    }
    else {
        // arrive at following target
        Vector3d leaderFollowingArrivalTarget = getFollowingTarget();


        Behaviors behaviors ({
                                 new Behavior_Arrive(leaderFollowingArrivalTarget, 40, m_agentId),
                                 new Behavior_Separation(30, m_agentId),
                                 new Behavior_Alignment(m_leaderId, m_agentId),
                                 new Behavior_SelfAlignment(m_agentId)
                             });
        return applyBehaviors(behaviors);
    }

}

Vector3d Behavior_LeaderFollowing::getFollowingTarget()
{
    MobilityData leaderMobility = leadersMobilityData();
    Vector3d currentDirection = getLeadersDirection();
    return leaderMobility.position +
            currentDirection * (-m_basicOffsetFromLeader_m);
}

Vector3d Behavior_LeaderFollowing::getLeadersDirection()
{
    MobilityData leaderMobility = leadersMobilityData();
    Orientation3d currentOrientation = leaderMobility.orientation;
    Vector3d direction = Vector3d::fromSphere(
                currentOrientation.pitch(),
                currentOrientation.yaw(),
                1.0
                ).truncated(4);

    return direction;
}

bool Behavior_LeaderFollowing::inLeadersFrontSpace()
{
    MobilityData leaderMobility = leadersMobilityData();
    Vector3d currentPosition = getAgent()->getPosition();
    Vector3d leadersDirection = getLeadersDirection();

    double angle = Vector3d::angle(leadersDirection, getAgent()->getPosition() - leaderMobility.position);
    if (
            abs(currentPosition.z - leaderMobility.position.z) > 3 ||
            angle > 90
            ) {
      return false;
    }


    // compute rectangle vertices
    Vector3d baseLeft = leaderMobility.position +
            leadersDirection.rotateLeft() * m_leaderFrontspaceWidth_m/2;

    Vector3d baseRight = leaderMobility.position +
            leadersDirection.rotateRight() * m_leaderFrontspaceWidth_m/2;

    Vector3d topLeft = baseLeft + leadersDirection * m_leaderFrontspaceHeight_m;

    Vector3d topRight = baseRight + leadersDirection * m_leaderFrontspaceHeight_m;


    // determine projected rectangle boundaries
    double x_max, x_min, y_max, y_min;
    std::vector<double> x_values ={baseLeft.x, baseRight.x, topLeft.x, topRight.x};
    std::vector<double> y_values ={baseLeft.y, baseRight.y, topLeft.y, topRight.y};
    x_max = *std::max_element(x_values.begin(), x_values.end());
    x_min = *std::min_element(x_values.begin(), x_values.end());
    y_max = *std::max_element(y_values.begin(), y_values.end());
    y_min = *std::min_element(y_values.begin(), y_values.end());

    return (
            currentPosition.x > x_min && currentPosition.x < x_max &&
            currentPosition.y > y_min && currentPosition.y < y_max
            );
}

MobilityData Behavior_LeaderFollowing::leadersMobilityData()
{
    MobilityData data = getAgent()->getLocalVehicleManager()
            ->getMobilityData().at(m_leaderId);

    return data;
}


} // namespace LIMoSim
