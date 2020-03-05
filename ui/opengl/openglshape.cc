#include "openglshape.h"

namespace OpenGL
{

OpenGLShape::OpenGLShape(const QColor &_color) :
    m_color(_color),
    m_limit(-1)
{

}


/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void OpenGLShape::addVertex(const QVector3D &_vertex)
{
    m_vertices << _vertex;

    while(m_limit>0 && m_vertices.size()>m_limit)
        m_vertices.pop_front();
}

void OpenGLShape::setColor(const QColor &_color)
{
    m_color = _color;
}

void OpenGLShape::setLimit(int _limit)
{
    m_limit = _limit;
}

const QVector<QVector3D>& OpenGLShape::getVertices() const
{
    return m_vertices;
}

QColor OpenGLShape::getColor() const
{
    return m_color;
}

}
