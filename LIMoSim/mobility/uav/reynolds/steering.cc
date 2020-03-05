#include "steering.h"

namespace LIMoSim
{

Steering::Steering():
    orientation(Orientation3d(0)),
    position(Vector3d())
{}

Steering::Steering(Orientation3d _orientation, Vector3d _position):
    orientation(_orientation),
    position(_position)
{}

Steering operator+(Steering _lhs, const Steering &_rhs)
{
    return Steering(_lhs.orientation + _rhs.orientation,
                    _lhs.position + _rhs.position);
}

Steering operator*(Steering _lhs, double _rhs)
{
    return Steering(_lhs.orientation * _rhs, _lhs.position * _rhs);
}

}
