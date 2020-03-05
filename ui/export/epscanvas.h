#ifndef LIMOSIM_EPSCANVAS_H
#define LIMOSIM_EPSCANVAS_H

#include "ui/canvas.h"
#include "epsdocument.h"
#include <QPointF>
#include "ui/opengl/openglrenderer.h"
#include "ui/opengl/earclipping.h"
#include <QMultiMap>
#include "plane.h"

using namespace OpenGL;

namespace LIMoSim
{

class EpsCanvas : public Canvas
{
public:
    EpsCanvas();

    //
    void init();
    void save(const QString &_path);
    void renderPlane(const Plane &_plane);

    // Canvas
    void drawNode(Node *_node);
    void drawRoadSegment(RoadSegment *_segment);
    void drawBuilding(Building *_building);

    //
    void drawPolygon(const QVector<QVector3D> &_polygon, const QVector3D &_normal, const QColor &_color);
    QVector<QVector3D> findPolygon(const std::vector<Node*> &_nodes);
    void setOpenGLRenderer(OpenGLRenderer *_gl);

private:
    QVector3D toVector(const Vector3d &_vector);

private:
    EpsDocument m_document;
    OpenGLRenderer *p_gl;

    QMultiMap<float, Plane> m_planes;
};

}

#endif // LIMOSIM_EPSCANVAS_H

