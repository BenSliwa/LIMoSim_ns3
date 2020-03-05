#ifndef LIMOSIM_WGS84_H
#define LIMOSIM_WGS84_H

#include "LIMoSim/world/vector3d.h"

// https://www.movable-type.co.uk/scripts/latlong.html

namespace LIMoSim
{

class WGS84
{
public:
    WGS84();

    static double computeDistance(const Vector3d &_from, const Vector3d &_to);
    static Vector3d computeOffset(const Vector3d &_position, const Vector3d &_origin);
};

}

#endif // LIMOSIM_WGS84_H
