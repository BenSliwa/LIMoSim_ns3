#ifndef LIMOSIM_BEHAVIOR_H
#define LIMOSIM_BEHAVIOR_H

#include <string>
#include "LIMoSim/mobility/uav/reynolds/reynoldsmodel.h"
#include "LIMoSim/mobility/uav/reynolds/steering.h"

namespace LIMoSim {

class UAV;
class Behavior;
class ReynoldsModel;

typedef std::vector<Behavior*> Behaviors;
typedef std::vector<double> Weights;

class Behavior {
public:
    Behavior(std::string _name, std::string _agentId = "");
    virtual ~Behavior();

    virtual Steering apply() = 0;

    //
    UAV* getAgent();
    std::string getName();
    //
    virtual void setAgent(std::string _agentId);

protected:
    std::string m_agentId;
    std::string setName(std::string _name);

    static Steering applyBehaviors(Behaviors _behaviors, bool _cleanup = true);
    Steering applyNormalizedBehaviors(Behaviors _behaviors, bool _cleanUpBehaviors = true);
    Steering applyNormalizedWeightedBehaviors(Behaviors _behaviors, Weights _weights, bool _cleanUpBehaviors = true);

    static Orientation3d computeDesiredOrientationVelocity(Vector3d _desiredVelocity, Orientation3d _currentOrientation);
    Orientation3d computeOrientationArrivalSteering(Orientation3d _desiredOrientationVelocity, Orientation3d _currentOrientationVelocity);
private:
    std::string m_name;
};

typedef std::vector<Behavior*> Behaviors;

}

#endif // LIMOSIM_BEHAVIOR
