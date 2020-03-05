#include "orientation3d.h"
#include "math.h"
#include "LIMoSim/world/worldutils.h"

namespace LIMoSim {

Orientation3d::Orientation3d(double _pitch, double _roll, double _yaw):
    Vector3d(_pitch,_roll,_yaw)
{

}

Orientation3d::Orientation3d(const Vector3d &_ref):
    Vector3d(_ref)
{

}

Orientation3d Orientation3d::fromDirection(Vector3d _direction)
{
    double cosinus = _direction.x / sqrt(_direction.x*_direction.x + _direction.y*_direction.y);
    Orientation3d orientation (
                Vector3d::toGrad(acos(_direction.z / _direction.norm())),
                0, // no roll for now
                normalizeAngle(Vector3d::toGrad(atan(_direction.y / _direction.x)) + ((cosinus < 0) ? 180 : 0))
                );
    return orientation;
}

double Orientation3d::pitch()
{
    return this->x;
}

double Orientation3d::roll()
{
    return this->y;
}

double Orientation3d::yaw()
{
    return this->z;
}

void Orientation3d::setPitch(double _pitch)
{
    this->x = _pitch;
}

void Orientation3d::setRoll(double _roll)
{
    this->y = _roll;
}

void Orientation3d::setYaw(double _yaw)
{
    this->z = _yaw;
}

Orientation3d operator+(Orientation3d _lhs, const Vector3d &_rhs)
{
    return Orientation3d(_lhs.x+_rhs.x, _lhs.y+_rhs.y, _lhs.z+_rhs.z);
}

Orientation3d operator-(Orientation3d _lhs, const Vector3d &_rhs)
{
    return Orientation3d(_lhs.x-_rhs.x, _lhs.y-_rhs.y, _lhs.z-_rhs.z);
}

} // namespace LIMoSim
