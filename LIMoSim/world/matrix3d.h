#ifndef MATRIX3D_H
#define MATRIX3D_H

#include "vector3d.h"
#include <vector>

namespace LIMoSim
{
class Matrix3d
{

public:
    Matrix3d inv();
    Vector3d cols(int _i);

    Matrix3d(
            double _a = 0, double _b = 0, double _c = 0,
            double _d = 0, double _e = 0, double _f = 0,
            double _g = 0, double _h = 0, double _i = 0);
    Matrix3d(Vector3d i, Vector3d j, Vector3d k);
    double a,b,c,d,e,f,g,h,i;
};

// operators

Matrix3d operator *(double _lhs, Matrix3d _rhs);
Matrix3d operator *(Matrix3d _lhs, double _rhs);
Vector3d operator *(Matrix3d _lhs, Vector3d _rhs);

}
#endif // MATRIX3D_H
