#include "epscanvas.h"


namespace LIMoSim
{

EpsCanvas::EpsCanvas() :
    Canvas(),
    p_gl(0)
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void EpsCanvas::init()
{
    m_document.init(2000, 2000);
}

void EpsCanvas::save(const QString &_path)
{
    // draw all planes
    QMultiMap<float, Plane>::iterator it;
    for(it=m_planes.begin(); it!=m_planes.end(); it++)
        renderPlane(it.value());

    //
    m_document.save(_path);

    // clean up
    m_planes.clear();
}

void EpsCanvas::renderPlane(const Plane &_plane)
{
    QVector<QVector3D> vertices = _plane.getVertices();
    for(int i=0; i<vertices.size(); i++)
    {
        QVector3D v = vertices.at(i);
        QPointF p(v.x() * 1200, v.y() * 700);
        p += QPointF(1000, 1000);

        if(i==0)
            m_document.moveTo(p);
        else
            m_document.lineTo(p);
    }

    m_document.setColor(_plane.getColor());
    m_document.fill();
}

void EpsCanvas::drawNode(Node *_node)
{

}

void EpsCanvas::drawRoadSegment(RoadSegment *_segment)
{
    Endpoint *from = _segment->getFromEndpoint();
    Endpoint *to = _segment->getToEndpoint();

    QVector<QVector3D> polygon;
    polygon << toVector(from->leftVertex);
    polygon << toVector(from->rightVertex);
    polygon << toVector(to->rightVertex);
    polygon << toVector(to->leftVertex);

    drawPolygon(polygon, QVector3D(0, 0, 1), "1c1c1c");
}

void EpsCanvas::drawBuilding(Building *_building)
{
    //
    QVector<QVector3D> ground = findPolygon(_building->getNodes());
    QVector3D height(0, 0, _building->getHeight());
    bool clockWise = EarClipping::isClockWise(ground);
    QVector<QVector3D> top;
    for(int i=0; i<ground.size(); i++)
        top << ground.at(i) + height;

    // draw all side planes
    int s = ground.size();
    for(int i=0; i<s; i++)
    {
        QVector3D v0 = top.at(i);
        QVector3D v1 = top.at((i+1)%s);
        QVector3D v0_l = ground.at(i);
        QVector3D v1_l = ground.at((i+1)%s);

        QVector3D dir = (v1-v0).normalized();
        QVector3D n(dir.y(), dir.x(), dir.z());

        if(clockWise)
            n.setY(n.y()*-1);
        else
            n.setX(n.x()*-1);

        // skip the rendering if the object is not visible (n-based)


        QVector<QVector3D> polygon = QVector<QVector3D>() << v0 << v1 << v1_l << v0_l;
        drawPolygon(polygon, n, "green");


        // visualize normal vectors if needed
        /*
        QVector3D p = (v1+v0)/2;
        p.setZ(height.z());
        float w = 1;
        float l = 5;
        QVector<QVector3D> polyN = QVector<QVector3D>() << p+dir*w << p-dir*w << p-dir*w+l*n << p+dir*w+l*n;
        drawPolygon(polyN, n, "blue");*/
    }

    // draw the top plane
    drawPolygon(top, QVector3D(0, 0, 1), "green");
}

void EpsCanvas::drawPolygon(const QVector<QVector3D> &_polygon, const QVector3D &_normal, const QColor &_color)
{
    QMatrix4x4 projection = p_gl->computeProjectionMatrix();
    QMatrix4x4 modelView = p_gl->computeModelViewMatrix();

    // blackface culling: do not draw elements that face away from the viewing direction
    QVector3D view = (p_gl->toNDC(_polygon.first()) - p_gl->getEye()).normalized();
    if(QVector3D::dotProduct(view, _normal)>=0)
       return;

    Plane plane;
    for(int i=0; i<_polygon.size(); i++)
    {
        QVector3D vertex = _polygon.at(i);
        vertex = p_gl->toNDC(vertex);

        QVector4D v = vertex.toVector4D(); v.setW(1);
        QVector3D c = p_gl->computeColor(_color);

        v = (projection * modelView * v);

        QVector3D v0 = v.toVector3D() / v.w();

        //
        plane.setColor(p_gl->computeColor(vertex, _normal, c));
        plane.addVertex(v0);
    }
    m_planes.insert(plane.computeZ(), plane);
}

QVector<QVector3D> EpsCanvas::findPolygon(const std::vector<Node*> &_nodes)
{
    QVector<QVector3D> polygon;
    for(int i=0; i<_nodes.size(); i++)
    {
        Vector3d p = _nodes.at(i)->getPosition();
        polygon << QVector3D(p.x, p.y, p.z);
    }
    return polygon;
}

void EpsCanvas::setOpenGLRenderer(OpenGLRenderer *_gl)
{
    p_gl = _gl;
}

QVector3D EpsCanvas::toVector(const Vector3d &_vector)
{
    return QVector3D(_vector.x, _vector.y, _vector.z);
}

}
