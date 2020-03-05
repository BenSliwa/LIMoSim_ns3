#ifndef LIMOSIM_VECTORRENDERER_H
#define LIMOSIM_VECTORRENDERER_H

#include "rectangle.h"
#include "epsdocument.h"
#include <QMatrix4x4>
#include <QPointF>

namespace OpenGL
{

class VectorRenderer
{
public:
    VectorRenderer();

    void saveScreenshot(const QString &_path, const QMatrix4x4 &_projection, const QMatrix4x4 &_modelView);

    QVector3D lookAt(const QVector3D &_vertex, const QMatrix4x4 &_projection, const QMatrix4x4 &_modelView);
    QPointF toPoint(const QVector3D &_vertex);

private:
    double m_width;
    double m_height;
};

}


#endif // LIMOSIM_VECTORRENDERER_H
