#include "vector3d.h"
#include <math.h>
#include <iostream>
#include <iomanip>
#include <algorithm>

namespace LIMoSim
{

Vector3d::Vector3d(double _x, double _y, double _z) :
    x(_x),
    y(_y),
    z(_z)
{

}

Vector3d Vector3d::fromSphere(double _theta, double _phi, double _r)
{
    double theta = toRad(_theta);
    double phi = toRad(_phi);

    return Vector3d(_r*sin(theta)*cos(phi), _r*sin(theta)*sin(phi), _r*cos(theta));
}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

double Vector3d::norm() const
{
    return sqrt(x*x+y*y+z*z);
}

Vector3d Vector3d::normed() const
{
    return *this/norm();
}

Vector3d Vector3d::truncated(int _decimals) const
{
    double x = pow(10,-_decimals) * (int)(this->x * pow(10,_decimals));
    double y = pow(10,-_decimals) * (int)(this->y * pow(10,_decimals));
    double z = pow(10,-_decimals) * (int)(this->z * pow(10,_decimals));
    return Vector3d(x, y, z);

}

Vector3d Vector3d::rotateRight()
{
    return Vector3d(y, -x, z);
}

Vector3d Vector3d::rotateLeft()
{
    return Vector3d(-y, x, z);
}

double Vector3d::computePhi()
{
    return toGrad(atan2(y, x));
}

Vector3d Vector3d::rotateTheta(double _offset)
{
    return Vector3d(x*cos(_offset) -y*sin(_offset), x*sin(_offset) + y*cos(_offset), z);
}

double Vector3d::computeAngleDifference(double _from, double _to)
{
    double d = _to - _from;
    if(fabs(d)>180)
    {
        d = std::min(_from, _to) + 360 - std::max(_from, _to);
        if(_from<_to)
            d *= -1;
    }

    return d;
}

double Vector3d::toRad(double _grad)
{
    return _grad/180*3.141592654;
}

double Vector3d::toGrad(double _rad)
{
    return _rad*180/3.141592654;
}


Vector3d Vector3d::cross(Vector3d u, Vector3d v)
{
    return Vector3d(
                u.y*v.z - u.z*v.y,
                u.z*v.x - u.x*v.z,
                u.x*v.y - u.y*v.x
                );
}

double Vector3d::det(Vector3d u, Vector3d v, Vector3d w)
{
    return u.x * (v.y*w.z-v.z*w.y) - u.y*(v.x*w.z-v.z*w.x) + u.z*(v.x*w.y-v.y*w.x);
}

double Vector3d::dot(Vector3d u, Vector3d v)
{
    return u.x*v.x + u.y*v.y + u.z*v.z;
}

double Vector3d::angle(Vector3d u, Vector3d v)
{
    return toGrad(acos((u.x*v.x + u.y+v.y + u.z*v.z) /(u.norm() * v.norm())));
}

Vector3d Vector3d::sign()
{
    return Vector3d(
                x >= 0 ? 1 : -1,
                y >= 0 ? 1 : -1,
                z >= 0 ? 1 : -1
                );
    // return cDiv((*this), cAbs());
}

Vector3d Vector3d::cAbs()
{
    return Vector3d( std::abs(x), std::abs(y), std::abs(z));
}

Vector3d Vector3d::cMin(Vector3d u, Vector3d v)
{
    return Vector3d (std::min(u.x, v.x), std::min(u.y, v.y), std::min(u.z, v.z));
}

Vector3d Vector3d::cMult(Vector3d u, Vector3d v)
{
    return Vector3d (u.x * v.x, u.y * v.y, u.z * v.z);
}

Vector3d Vector3d::cDiv(Vector3d u, Vector3d v)
{
    return Vector3d (u.x / v.x, u.y / v.y, u.z / v.z);
}

std::string Vector3d::toString(std::string sep) const
{
    std::stringstream stream;
    stream  << std::fixed << std::setw(4) << std::setprecision(3)
            << x << sep << y << sep << z;

    return stream.str();
}

void Vector3d::info()
{
    std::cout << toString() << std::endl;
}

std::ostream &Vector3d::serialize(std::ostream &_os) const
{
    _os << x << std::endl;
    _os << y << std::endl;
    _os << z << std::endl;
    return _os;
}

std::istream &Vector3d::deserialize(std::istream &_is)
{
    _is >> x;
    _is >> y;
    _is >> z;
    return _is;
}


/*************************************
 *              OPERATORS            *
 ************************************/

Vector3d operator+(Vector3d _lhs, const Vector3d &_rhs)
{
    return Vector3d(_lhs.x+_rhs.x, _lhs.y+_rhs.y, _lhs.z+_rhs.z);
}

Vector3d operator-(Vector3d _lhs, const Vector3d &_rhs)
{
    return Vector3d(_lhs.x-_rhs.x, _lhs.y-_rhs.y, _lhs.z-_rhs.z);
}

Vector3d operator*(Vector3d _lhs, double _rhs)
{
    return Vector3d(_lhs.x*_rhs, _lhs.y*_rhs, _lhs.z*_rhs);
}

Vector3d operator/(Vector3d _lhs, double _rhs)
{
    return Vector3d(_lhs.x/_rhs, _lhs.y/_rhs, _lhs.z/_rhs);
}

double operator*(Vector3d _lhs, const Vector3d &_rhs)
{
    return _lhs.x*_rhs.x + _lhs.y*_rhs.y + _lhs.z*_rhs.z;
}

std::ostream &operator<<(std::ostream & _os, const Vector3d &_rhs)
{
    _os << _rhs.x << " " << _rhs.y << " " << _rhs.z << " ";
    return _os;
}

std::istream &operator>>(std::istream &_is, Vector3d &_rhs)
{
    _is >> _rhs.x >> _rhs.y >> _rhs.z;
    return _is;
}

bool operator==(const Vector3d _lhs, const Vector3d &_rhs)
{
    return _lhs.x == _rhs.x && _lhs.y == _rhs.y && _lhs.z == _rhs.z;
}

bool operator!=(const Vector3d _lhs, const Vector3d &_rhs)
{
    return !(_lhs == _rhs);
}


}
