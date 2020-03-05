#ifndef LIMOSIM_VECTOR3D_H
#define LIMOSIM_VECTOR3D_H

#include <sstream>

namespace LIMoSim
{

class Vector3d
{
public:
    Vector3d(double _x=0, double _y=0, double _z=0);
    static Vector3d fromSphere(double _phi, double _theta, double _r);

    double norm() const;
    Vector3d normed() const;
    Vector3d truncated(int _decimals) const;
    Vector3d rotateRight();
    Vector3d rotateLeft();
    double computePhi();
    static double computeAngleDifference(double _from, double _to);
    Vector3d rotateTheta(double _offset);

    //
    static double toRad(double _grad);
    static double toGrad(double _rad);

    //
    static Vector3d cross(Vector3d u, Vector3d v);
    static double det(Vector3d u, Vector3d v, Vector3d w);
    static double dot(Vector3d u, Vector3d v);
    static double angle(Vector3d u, Vector3d v);

    //
    Vector3d sign();

    //
    Vector3d cAbs();
    static Vector3d cMin(Vector3d u, Vector3d v);
    static Vector3d cMult(Vector3d u, Vector3d v);
    static Vector3d cDiv(Vector3d u, Vector3d v);

    //
    virtual std::string toString(std::string sep=",") const;
    void info();



    std::ostream& serialize(std::ostream& os) const;
    std::istream& deserialize(std::istream& _is);


public:
    double x;
    double y;
    double z;
};

// operators
Vector3d operator+(Vector3d _lhs, const Vector3d &_rhs);
Vector3d operator-(Vector3d _lhs, const Vector3d &_rhs);
Vector3d operator*(Vector3d _lhs, double _rhs);
Vector3d operator/(Vector3d _lhs, double _rhs);
double operator*(Vector3d _lhs, const Vector3d &_rhs);
bool operator==(const Vector3d _lhs, const Vector3d &_rhs);
bool operator!=(const Vector3d _lhs, const Vector3d &_rhs);
std::ostream& operator<<(std::ostream& _os, const Vector3d &_rhs);
std::istream& operator>>(std::istream& _is, Vector3d &_rhs);

}

#endif // VECTOR3D
