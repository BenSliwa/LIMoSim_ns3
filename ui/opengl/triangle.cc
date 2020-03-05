#include "triangle.h"
#include <QDebug>

namespace OpenGL
{

Triangle::Triangle(const QVector3D &_first, const QVector3D &_second, const QVector3D &_third) :
    m_first(_first),
    m_second(_second),
    m_third(_third)
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

bool Triangle::isCenterConvex(bool _clockwise)
{
    if(_clockwise)
        return (sign(m_second, m_first, m_third) < 0.0f);
    return (sign(m_second, m_third, m_first) < 0.0f);

}

bool Triangle::isInside(const QVector3D &_vertex)
{
    bool b1, b2, b3;
    b1 = sign(_vertex, m_first, m_second) < 0.0f;
    b2 = sign(_vertex, m_second, m_third) < 0.0f;
    b3 = sign(_vertex, m_third, m_first) < 0.0f;

    return ((b1 == b2) && (b2 == b3));
}

float Triangle::sign(const QVector3D &_v0, const QVector3D &_v1, const QVector3D &_v2)
{
    return (_v0.x()-_v2.x()) * (_v1.y()-_v2.y()) - (_v1.x()-_v2.x()) * (_v0.y()-_v2.y());
}

void Triangle::info()
{
    qDebug() << m_first << m_second << m_third;
}

}
