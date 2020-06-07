#include "openglcanvas.h"
#include "openglshape.h"

namespace OpenGL
{

OpenGLCanvas::OpenGLCanvas() :
    Canvas(),
    p_renderer(0)
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void OpenGLCanvas::drawNode(Node *_node)
{
    if(p_world->getIntersection(_node))
        drawCube(_node->getPosition(), 1.5, "yellow");
    else
        drawCube(_node->getPosition(), 1.5, "red");
}

void OpenGLCanvas::drawRoadSegment(RoadSegment *_segment)
{
    Endpoint *from = _segment->getFromEndpoint();
    Endpoint *to = _segment->getToEndpoint();


    std::vector<LaneSegment*> forwardLanes = _segment->getLanes(LANE_TYPE::FORWARD);
    for(unsigned int j=0; j<forwardLanes.size(); j++)
    {
        LaneSegment* lane = forwardLanes.at(j);

        Vector3d to = lane->getEnd();
        Vector3d from = lane->getStart();
        Vector3d dir = (to-from).normed();
        double w = 0.25;

        /*
        OpenGL::Shape s("white");
        s.vertices << ndcFromVector(from + dir.rotateLeft() * w) << ndcFromVector(from + dir.rotateRight() * w);
        s.vertices << ndcFromVector(to + dir.rotateRight() * w) << ndcFromVector(to + dir.rotateLeft() * w);
        p_renderer->drawShape(s);*/
    }


    std::vector<LaneSegment*> backwardLanes = _segment->getLanes(LANE_TYPE::BACKWARD);
    for(unsigned int j=0; j<backwardLanes.size(); j++)
    {
        LaneSegment* lane = backwardLanes.at(j);

        Vector3d to = lane->getEnd();
        Vector3d from = lane->getStart();
        Vector3d dir = (to-from).normed();
        double w = 0.25;

        /*
        OpenGL::Shape s("green");
        s.vertices << ndcFromVector(from + dir.rotateLeft() * w) << ndcFromVector(from + dir.rotateRight() * w);
        s.vertices << ndcFromVector(to + dir.rotateRight() * w) << ndcFromVector(to + dir.rotateLeft() * w);
        p_renderer->drawShape(s);*/
    }

    OpenGLShape shape("#1c1c1c");
    shape.addVertex(QVector3D(fromVector(from->leftVertex)));
    shape.addVertex(QVector3D(fromVector(from->rightVertex)));
    shape.addVertex(QVector3D(fromVector(to->rightVertex)));
    shape.addVertex(QVector3D(fromVector(to->leftVertex)));
    p_renderer->drawCube(shape, 0);





}

QVector3D OpenGLCanvas::fromVector(const Vector3d &_vector)
{
    return QVector3D(_vector.x, _vector.y, _vector.z);
}

void OpenGLCanvas::drawCube(const Vector3d &_position, float _width, const QColor &_color)
{
    float x = _position.x;
    float y = _position.y;
    float z = _position.z;
    float w = _width;

    OpenGLShape shape(_color);
    shape.addVertex(QVector3D(x-w, y-w, z));
    shape.addVertex(QVector3D(x-w, y+w, z));
    shape.addVertex(QVector3D(x+w, y+w, z));
    shape.addVertex(QVector3D(x+w, y-w, z));
    p_renderer->drawPolygon(shape, 0.1);
}

void OpenGLCanvas::drawBuilding(Building *_building)
{
    std::vector<Node*> nodes = _building->getNodes();
    if(nodes.size()>1)
    {
        OpenGLShape shape("darkgray");
        for(unsigned int i=0; i<nodes.size()-1; i++)
        {
            Vector3d position = nodes.at(i)->getPosition();
            shape.addVertex(fromVector(position));
        }
        p_renderer->drawPolygon(shape, _building->getHeight());
    }
}

void OpenGLCanvas::setRenderer(OpenGLRenderer *_renderer)
{
    p_renderer = _renderer;
}

OpenGLShape OpenGLCanvas::buildTriangle(const Vector3d &_center, double _direction, double _width, double _length)
{
    OpenGLShape shape("white");

    Vector3d dir = Vector3d::fromSphere(90, _direction, _length/2);
    Vector3d front = _center + dir;
    Vector3d foot = _center - dir;

    dir = dir.rotateRight().normed() * _width/2;
    Vector3d left = foot - dir;
    Vector3d right = foot + dir;

    shape.addVertex(fromVector(front));
    shape.addVertex(fromVector(left));
    shape.addVertex(fromVector(right));

    p_renderer->drawPolygon(shape, 0.1, false);

    return shape;
}

}
