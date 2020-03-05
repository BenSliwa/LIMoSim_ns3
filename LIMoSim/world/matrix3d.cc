#include "matrix3d.h"

namespace LIMoSim {

Matrix3d Matrix3d::inv()
{

    double det = Vector3d::det(
                cols(0),
                cols(1),
                cols(2)
                );
    Matrix3d inverse (
                e*i - f*h, c*h - b*i, b*f - c*e,
                f*g - d*i, a*i - c*g, c*d - a*f,
                d*h - e*g, b*g - a*h, a*e - b*d
                );
    if (!det) {
        return Matrix3d();
    }
    inverse = (1/det) * inverse;
    return inverse;
}

Vector3d Matrix3d::cols(int _i)
{
    if (_i < 0 || _i > 2) {
        return Vector3d();
    }

    Vector3d col;

    switch (_i) {
    case 0:
        col = Vector3d(a, d, g);
        break;
    case 1:
        col = Vector3d(b, e, h);
        break;
    case 2:
        col = Vector3d(c, f, i);
        break;
    }

    return col;
}

Matrix3d::Matrix3d(
        double _a, double _b, double _c,
        double _d, double _e, double _f,
        double _g, double _h, double _i
        ):
    a(_a), b(_b), c(_c),
    d(_d), e(_e), f(_f),
    g(_g), h(_h), i(_i)
{

}

Matrix3d::Matrix3d(Vector3d i, Vector3d j, Vector3d k):
    a(i.x), b(j.x), c(k.x),
    d(i.y), e(j.y), f(k.y),
    g(i.z), h(j.z), i(k.z)
{

}

Matrix3d operator *(double _lhs, Matrix3d _rhs)
{
    return Matrix3d(
                _rhs.a*_lhs, _rhs.b*_lhs, _rhs.c*_lhs,
                _rhs.d*_lhs, _rhs.e*_lhs, _rhs.f*_lhs,
                _rhs.g*_lhs, _rhs.h*_lhs, _rhs.i*_lhs
                );
}

Matrix3d operator *(Matrix3d _lhs, double _rhs)
{
    return Matrix3d(
                _lhs.a*_rhs, _lhs.b*_rhs, _lhs.c*_rhs,
                _lhs.d*_rhs, _lhs.e*_rhs, _lhs.f*_rhs,
                _lhs.g*_rhs, _lhs.h*_rhs, _lhs.i*_rhs
                );
}

Vector3d operator *(Matrix3d _lhs, Vector3d _rhs)
{
    return Vector3d(
                (_lhs.a * _rhs.x) + (_lhs.b * _rhs.y) + (_lhs.c * _rhs.z),
                (_lhs.d * _rhs.x) + (_lhs.e * _rhs.y) + (_lhs.f * _rhs.z),
                (_lhs.g * _rhs.x) + (_lhs.h * _rhs.y) + (_lhs.i * _rhs.z)
                );
}

}
