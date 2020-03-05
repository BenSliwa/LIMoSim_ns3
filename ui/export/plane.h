#ifndef LIMOSIM_PLANE_H
#define LIMOSIM_PLANE_H

#include <QVector>
#include <QVector3D>
#include <QColor>

namespace LIMoSim
{

class Plane
{
public:
    Plane();

    void addVertex(const QVector3D &_vertex);
    float computeZ();

    //
    void setColor(const QColor &_color);

    QColor getColor() const;
    const QVector<QVector3D>& getVertices() const;



private:
    QVector<QVector3D> m_vertices;
    QColor m_color;

};

}

#endif // LIMOSIM_PLANE_H
