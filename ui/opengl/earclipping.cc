#include "earclipping.h"
#include <math.h>
#include <QDebug>

namespace OpenGL
{


EarClipping::EarClipping()
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

QVector<QVector3D> EarClipping::triangulate(const QVector<QVector3D> &_polygon)
{
    QVector<QVector3D> triangles;
    m_polygon = _polygon;
    for(int i=0; i<m_polygon.size(); i++)
        m_vertices << i;

    // the polygon direction needs to be known for the reflex/convex classification
    m_clockwise = isClockWise(_polygon);

    // apply the initialization steps
    classify();
    findEars();

    //
    int id, index, previous, next;
    while(m_ears.size()>0)
    {
        if(m_vertices.size()==3)
        {
            triangles << getVertex(m_vertices.at(0)) << getVertex(m_vertices.at(1)) << getVertex(m_vertices.at(2));
            break;
        }

        id = m_ears.first();

        index = m_vertices.indexOf(id);
        previous = getPrevious(id);
        next = getNext(id);
        removeEar(id);

        updateVertex(previous);
        updateVertex(next);

        triangles << getVertex(previous) << getVertex(id) << getVertex(next);
    }

    return triangles;
}

bool EarClipping::isClockWise(const QVector<QVector3D> &_polygon)
{
    float sum = 0;
    for(int i=0; i<_polygon.size(); i++)
    {
        QVector3D v0 = _polygon.at(i);
        QVector3D v1 = _polygon.at((i+1)%_polygon.size());
        sum += (v1.x()-v0.x())*(v1.y()+v0.y());
    }

    if(sum<0.0)
        return true;
    return false;
}

void EarClipping::classify()
{
    m_convex.clear();
    m_reflex.clear();

    // classify the vertices into reflex and convex
    for(int i=0; i<m_vertices.size(); i++)
    {
        int id = m_vertices.at(i);
        if(isConvex(id))
            m_convex << id;
        else
            m_reflex << id;
    }
}

void EarClipping::findEars()
{
    m_ears.clear();
    for(int i=0; i<m_convex.size(); i++)
    {
        int id = m_convex.at(i);
        if(isEar(id))
            m_ears << id;
    }
}

bool EarClipping::isConvex(int _id) // req: connvex
{
    Triangle triangle = getTriangle(_id);
    return triangle.isCenterConvex(m_clockwise);
}

bool EarClipping::isEar(int _id)
{
    Triangle triangle = getTriangle(_id);

    bool inside = false;
    for(int j=0; j<m_vertices.size(); j++) // test all other vertices
    {
        int k = m_vertices.at(j);
        if(k!=getPrevious(_id) && k!=_id && k!=getNext(_id))
        {
            QVector3D vertex = getVertex(m_vertices.at(j));
            if(triangle.isInside(vertex))
            {
                inside = true;
                break;
            }
        }
    }

    if(inside==0)
        return true;
    return false;
}

void EarClipping::updateVertex(int _id)
{
    if(isConvex(_id))
    {
        if(!m_convex.contains(_id))
        {
            m_convex << _id;
            m_reflex.removeAt(m_reflex.indexOf(_id));
        }
        if(isEar(_id) && !m_ears.contains(_id))
            m_ears << _id;
    }
}

void EarClipping::removeEar(int _id)
{
    m_vertices.removeAt(m_vertices.indexOf(_id));
    m_ears.removeAt(m_ears.indexOf(_id));
    m_convex.removeAt(m_convex.indexOf(_id));
}

QVector3D EarClipping::getVertex(int _id)
{
    return m_polygon.at(_id);
}

Triangle EarClipping::getTriangle(int _id)
{
    return Triangle(getVertex(getPrevious(_id)), getVertex(_id), getVertex(getNext(_id)));
}

int EarClipping::getPrevious(int _id)
{
    int index = m_vertices.indexOf(_id);
    return index>0 ? m_vertices.at(index-1) : m_vertices.last();
}

int EarClipping::getNext(int _id)
{
    int index = m_vertices.indexOf(_id);
    return index<m_vertices.size()-1 ? m_vertices.at(index+1) : m_vertices.first();
}

}
