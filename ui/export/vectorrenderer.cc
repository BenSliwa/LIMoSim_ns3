#include "vectorrenderer.h"
#include "epsdocument.h"

namespace OpenGL
{

VectorRenderer::VectorRenderer()
{

}

void VectorRenderer::saveScreenshot(const QString &_path, const QMatrix4x4 &_projection, const QMatrix4x4 &_modelView)
{
    LIMoSim::EpsDocument eps;
    eps.init(m_width, m_height);

    QVector3D origin(200, 200, 0);

    QVector<QVector3D> rect;
    rect << QVector3D(0, 0, 0) + origin;
    rect << QVector3D(50, 0, 0) + origin;
    rect << QVector3D(50, 50, 0) + origin;
    rect << QVector3D(0, 50, 0) + origin;

    eps.startPath();
    for(int i=0; i<rect.size(); i++)
    {
        QVector3D vertex0 = rect.at(i);
        QVector3D vertex1 = lookAt(vertex0, _projection, _modelView);

        qDebug() << vertex0 << vertex1;

        QPointF point = toPoint(vertex1);

        if(i==0)
            eps.moveTo(point);
        else
            eps.lineTo(point);
    }
    eps.setColor("green");
    eps.fill();

    eps.save(_path);
}

QVector3D VectorRenderer::lookAt(const QVector3D &_vertex, const QMatrix4x4 &_projection, const QMatrix4x4 &_modelView)
{
    QVector4D vertex = _vertex.toVector4D();
    vertex.setW(1);
    vertex = _projection * _modelView * vertex;

    // perspective division
    QVector3D result = vertex.toVector3D();// / vertex.w();

    // resize
   // result.setX(result.x() * m_width);
   // result.setY(result.y() * m_height);

    return result;
}

QPointF VectorRenderer::toPoint(const QVector3D &_vertex)
{
    return QPointF(_vertex.x(), _vertex.y());
}

}
