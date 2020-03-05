#include "behavior_cohesion2d.h"

namespace LIMoSim {

Behavior_Cohesion2D::Behavior_Cohesion2D(double _searchRadius, std::string _agentId):
    Behavior_Cohesion("Cohesion2D", _searchRadius, _agentId)
{

}

Steering Behavior_Cohesion2D::apply()
{
    // Almost same steering as parent
    // without steering on the z-axis
    auto steering = Behavior_Cohesion::apply();
    steering.position.z = 0;
    return steering;
}

Behavior_Cohesion2D::Behavior_Cohesion2D(std::string _name,
                                         double _searchRadius,
                                         std::string _agentId):
    Behavior_Cohesion(_name, _searchRadius, _agentId)
{

}

} // namespace LIMoSim
