#ifndef BEHAVIOR_DOCK_H
#define BEHAVIOR_DOCK_H

#include "LIMoSim/mobility/uav/reynolds/behavior.h"
#include "LIMoSim/mobility/uav/reynolds/behavior_arrive.h"

namespace LIMoSim {
namespace delivery {
namespace Behaviors {

class Behavior_Dock : public Behavior
{
public:
    Behavior_Dock(std::string _dockTargetId, std::string _agentId="");
    virtual ~Behavior_Dock();

    std::string getDockTargetId();
    Vehicle* getDockTarget();

    // Behavior interface
public:
    Steering apply();
    virtual void setAgent(std::string _agentId);

private:
    std::string m_dockTargetId;
    Vehicle * m_dockTarget;
    Behavior_Arrive m_b;

};

} // namespace Behaviors
} // namespace Delivery
} // namespace LIMoSim

#endif // BEHAVIOR_DOCK_H
