#ifndef OPENGL_VERTEX_H
#define OPENGL_VERTEX_H

#include <QString>

namespace OpenGL
{

class Vertex
{
public:
    Vertex(float _x = 0, float _y = 0, float _z = 0);

    QString toString() const;

public:
    float x;
    float y;
    float z;

};

}


#endif // OPENGL_VERTEX_H
