#ifndef BEHAVIOR_ALIGNMENT_H
#define BEHAVIOR_ALIGNMENT_H

#include <map>
#include "LIMoSim/mobility/uav/reynolds/behavior.h"

namespace LIMoSim {

class MobilityData;
class Behavior_Alignment : public Behavior
{
public:
    Behavior_Alignment(double _searchRadius = 200.0, std::string _agentId="");
    Behavior_Alignment(std::string _referenceAgentId, std::string _agentId="");

    // Behavior interface
    Steering apply();

    UAV* getReferenceAgent();

protected:
    std::map<std::string, MobilityData> findNearByUAVs();

private:
    double m_searchRadius;
    std::string m_referenceAgentId;
};

} // namespace LIMoSim

#endif // BEHAVIOR_ALIGNMENT_H
