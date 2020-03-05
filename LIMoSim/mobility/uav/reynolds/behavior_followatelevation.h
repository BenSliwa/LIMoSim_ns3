#ifndef BEHAVIOR_FOLLOWATELEVATION_H
#define BEHAVIOR_FOLLOWATELEVATION_H


#include "LIMoSim/mobility/uav/reynolds/behavior.h"

namespace LIMoSim {

class UAV;

class Behavior_FollowAtElevation : public Behavior
{
public:
    Behavior_FollowAtElevation(
            std::string _targetAgentId,
            double _elevation = 20.0,
            double _followRadius = 40.0,
            bool _predictionEnabled = true,
            std::string _agentId = ""
            );
    ~Behavior_FollowAtElevation();

private:
    std::string m_targetAgentId;
    double m_elevation_m;
    double m_followRadius_m;
    bool m_predictionEnabled;
    Behavior *m_selfAlignment;

    Vector3d predictTargetPosition();

    Vector3d predictPosition(double _future);

    // Behavior interface
public:
    Steering apply();
    void setAgent(std::string _agentId);
};

} // namespace LIMoSim

#endif // BEHAVIOR_FOLLOWATELEVATION_H
