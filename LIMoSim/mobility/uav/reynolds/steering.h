#ifndef LIMOSIM_STEERING_H
#define LIMOSIM_STEERING_H

#include <vector>
#include "LIMoSim/world/vector3d.h"
#include "LIMoSim/world/orientation3d.h"

namespace LIMoSim
{

class Steering
{
public:
    Steering();

    Steering(Orientation3d _orientation, Vector3d _position);


    Orientation3d orientation;
    Vector3d position;
};

Steering operator+(Steering _lhs, const Steering &_rhs);
Steering operator*(Steering _lhs, double _rhs);

typedef std::vector<Steering> Steerings;
}

#endif // LIMOSIM_STEERING_H
