#ifndef LIMOSIM_FOLLOWINGMODEL_H
#define LIMOSIM_FOLLOWINGMODEL_H

#include "LIMoSim/mobility/car/car.h"
#include <math.h>

namespace LIMoSim
{

class FollowingModel
{
public:
    FollowingModel(Car *_car);

protected:
    Car *p_car;
};

}


#endif // LIMOSIM_FOLLOWINGMODEL_H
