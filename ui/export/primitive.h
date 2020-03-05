#ifndef LIMOSIM_PRIMITIVE_H
#define LIMOSIM_PRIMITIVE_H

#include <QVector3D>
#include <QVector4D>
#include <QVector>


namespace OpenGL
{

class Primitive
{
public:
    Primitive();

protected:
    QVector<QVector3D> m_vertices;
};


}


#endif // LIMOSIM_PRIMITIVE_H
