#ifndef LIMOSIM_OPENGLCANVAS_H
#define LIMOSIM_OPENGLCANVAS_H

#include "openglrenderer.h"
#include "ui/canvas.h"

using namespace LIMoSim;

namespace OpenGL
{

class OpenGLCanvas : public Canvas
{
public:
    OpenGLCanvas();

    // Canvas
    void drawNode(Node *_node);
    void drawRoadSegment(RoadSegment *_segment);
    void drawBuilding(Building *_building);

    //
    void setRenderer(OpenGLRenderer *_renderer);
    QVector3D fromVector(const Vector3d &_vector);
    void drawCube(const Vector3d &_position, float _width, const QColor &_color);

    OpenGLShape buildTriangle(const Vector3d &_center, double _direction, double _width, double _length);


private:
    OpenGLRenderer *p_renderer;
};

}

#endif // OPENGLCANVAS_H
