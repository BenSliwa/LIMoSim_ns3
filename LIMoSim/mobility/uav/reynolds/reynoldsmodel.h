#ifndef LIMOSIM_REYNOLDSMODEL_H
#define LIMOSIM_REYNOLDSMODEL_H

#include "LIMoSim/mobility/mobilitymodel.h"
#include "LIMoSim/mobility/uav/reynolds/locomotion.h"
#include "LIMoSim/mobility/uav/reynolds/locomotionupdate.h"

namespace LIMoSim {

class UAV;
class Behavior;

class ReynoldsModel: public MobilityModel
{
public:
    ReynoldsModel(
            std::string _agentId,
            Behavior* _behavior,
            Locomotion* _locomotion
            );
    virtual ~ReynoldsModel();

    UAV* getAgent();
    virtual Locomotion* getLocomotion();
    Behavior* getBehavior();
    std::string getBehaviorName();
    void setBehavior(Behavior *_behavior);
    LocomotionUpdate step(double _timeDelta_s);

protected:
    Steering applyBehavior(Behavior *_behavior);
    LocomotionUpdate applyLocomotion(Steering _steering, double _timeDelta_s);


private:
    Behavior* m_behavior;
    Locomotion* m_locomotion;
};

}


#endif // REYNOLDSMODEL_H
