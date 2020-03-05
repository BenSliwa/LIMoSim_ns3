#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <QVector3D>

namespace OpenGL
{

class Triangle
{
public:
    Triangle(const QVector3D &_first, const QVector3D &_second, const QVector3D &_third);

    bool isCenterConvex(bool _clockwise);
    bool isInside(const QVector3D &_vertex);
    float sign(const QVector3D &_v0, const QVector3D &_v1, const QVector3D &_v2);

    void info();

private:
    QVector3D m_first;
    QVector3D m_second;
    QVector3D m_third;
};

}

#endif // TRIANGLE_H
