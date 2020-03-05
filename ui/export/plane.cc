#include "plane.h"

namespace LIMoSim
{

Plane::Plane()
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void Plane::addVertex(const QVector3D &_vertex)
{
    m_vertices << _vertex;
}

float Plane::computeZ()
{
    float z = 0;
    for(int i=0; i<m_vertices.size(); i++)
        z += m_vertices.at(i).z();
    z /= -(float)m_vertices.size();


    return z;
}

void Plane::setColor(const QColor &_color)
{
    m_color = _color;
}

QColor Plane::getColor() const
{
    return m_color;
}

const QVector<QVector3D>& Plane::getVertices() const
{
    return m_vertices;
}

}
