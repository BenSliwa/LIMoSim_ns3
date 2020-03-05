#include "vertex.h"

namespace OpenGL
{

Vertex::Vertex(float _x, float _y, float _z) :
    x(_x),
    y(_y),
    z(_z)
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

QString Vertex::toString() const
{
    QString data = QString::number(x) + "," +  QString::number(y) + "," + QString::number(z);
    return data;
}

}
