#ifndef EARCLIPPING_H
#define EARCLIPPING_H

#include <QVector>
#include "triangle.h"

// https://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf

namespace OpenGL
{
struct Vertex
{
    QVector3D coords;
    int index;
};

class EarClipping
{
public:
    EarClipping();

    QVector<QVector3D> triangulate(const QVector<QVector3D> &_polygon);
    static bool isClockWise(const QVector<QVector3D> &_polygon);

    void classify();
    void findEars();

    bool isConvex(int _id);
    bool isEar(int _id);
    void updateVertex(int _id);
    void removeEar(int _id);

    QVector3D getVertex(int _id);
    Triangle getTriangle(int _id);

    int getPrevious(int _id);
    int getNext(int _id);

    // convex: the angle of the vertex to its neighbors is <180

private:
    QVector<QVector3D> m_polygon;

    QVector<int> m_vertices;
    QVector<int> m_reflex;
    QVector<int> m_convex;
    QVector<int> m_ears;

    bool m_clockwise;
};

}

#endif // EARCLIPPING_H
