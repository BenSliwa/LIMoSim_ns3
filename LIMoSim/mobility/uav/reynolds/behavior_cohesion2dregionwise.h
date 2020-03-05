#ifndef BEHAVIOR_COHESION2DREGIONWISE_H
#define BEHAVIOR_COHESION2DREGIONWISE_H

#include "behavior_cohesion2d.h"

namespace LIMoSim {

class Behavior_Cohesion2DRegionwise : public LIMoSim::Behavior_Cohesion2D
{
public:
    Behavior_Cohesion2DRegionwise(Vector3d _regionTopLeft, double _regionHeight, double _regionWidth, std::string _agentId="");

protected:
    virtual std::map<std::string, MobilityData> findNearByUAVs();
    bool isInBounds(Vector3d _position);

    Vector3d m_regionTopLeft;
    double m_regionHeight;
    double m_regionWidth;
};

} // namespace LIMoSim

#endif // BEHAVIOR_COHESION2DREGIONWISE_H
