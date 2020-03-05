#ifndef LOCOMOTIONUPDATE_H
#define LOCOMOTIONUPDATE_H

#include "LIMoSim/world/vector3d.h"
#include "LIMoSim/world/orientation3d.h"

namespace LIMoSim {

struct LocomotionUpdate {
    Orientation3d orientation;
    Orientation3d orientationVelocity;
    Vector3d position;
    Vector3d velocity;
    Vector3d acceleration;

    LocomotionUpdate(Orientation3d _orientation,
                     Orientation3d _orientationVelocity,
                     Vector3d _position,
                     Vector3d _speed,
                     Vector3d _acceleration):
        orientation(_orientation),
        orientationVelocity(_orientationVelocity),
        position(_position),
        velocity(_speed),
        acceleration(_acceleration)
    {}
};

}
#endif // LOCOMOTIONUPDATE_H
