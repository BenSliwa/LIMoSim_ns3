#ifndef ORIENTATION3D_H
#define ORIENTATION3D_H

#include "vector3d.h"

namespace LIMoSim {

class Orientation3d : public Vector3d
{
public:
    Orientation3d(double _pitch=90, double _roll=0, double _yaw=0);
    Orientation3d(Vector3d const & _ref);

    static Orientation3d fromDirection(Vector3d _direction);

    double pitch();
    double roll();
    double yaw();
    void setPitch(double _pitch);
    void setRoll(double _roll);
    void setYaw(double _yaw);
};

// operators
Orientation3d operator+(Orientation3d _lhs, const Vector3d &_rhs);
Orientation3d operator-(Orientation3d _lhs, const Vector3d &_rhs);

} // namespace LIMoSim

#endif // ORIENTATION3D_H
