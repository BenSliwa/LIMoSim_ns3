#ifndef LIMOSIM_UAV_H
#define LIMOSIM_UAV_H

#include "LIMoSim/mobility/vehicle.h"
#include "reynolds/uav_reynoldsmodel.h"

namespace LIMoSim
{
class UAV_ReynoldsModel;
class UAV : public Vehicle
{
    friend class Behavior;
    friend class Steering;
    friend class Locomotion;

public:
    UAV(const std::string &_id);

    // Reynoldsmodel related methods
    UAV_ReynoldsModel *getModel();
    void setAccelerationMax(double _accMax);
    void setBehavior(Behavior* _behavior);
    void setVelocityMax(double _velMax);

protected:

    void initialize();
    void handleEvent(Event *_event);
    void move(double _timeDelta_s);

private:
    Vector3d m_initialPosition;
    Vector3d generateNormedNoiseVector();
};

}

#endif // LIMOSIM_UAV_H
