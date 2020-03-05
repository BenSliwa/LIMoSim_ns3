#include "cube.h"

Cube::Cube() :
    OpenGLObject()
{

}

void Cube::create(const QVector3D &_pos, const QVector3D &_dim)
{
    GLfloat x = _dim.x();
    GLfloat y = _dim.y();
    GLfloat z = _dim.z();

    const GLfloat x1 = _pos.x();
    const GLfloat y1 = _pos.y();
    const GLfloat x2 = x1 + x;
    const GLfloat y2 = y1 + y;

    /*
    quad(x1, y1,
         x1, y2,
         x2, y2,
         x2, y1, 0, z);

    if(z>0.0)
    {
        extrude(x1, y1, x1, y2, 0, z);
        extrude(x1, y2, x2, y2, 0, z);
        extrude(x2, y2, x2, y1, 0, z);
        extrude(x2, y1, x1, y1, 0, z);
    }*/
}
