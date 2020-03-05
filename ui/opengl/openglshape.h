#ifndef OPENGLSHAPE_H
#define OPENGLSHAPE_H

#include <QVector>
#include <QVector3D>
#include <QColor>

namespace OpenGL
{

class OpenGLShape
{
public:
    OpenGLShape(const QColor &_color);

    void addVertex(const QVector3D &_vertex);

    void setColor(const QColor &_color);
    void setLimit(int _limit);
    const QVector<QVector3D>& getVertices() const;
    QColor getColor() const;


private:
    QVector<QVector3D> m_vertices;
    QColor m_color;
    int m_limit;
};

}

#endif // OPENGLSHAPE_H
