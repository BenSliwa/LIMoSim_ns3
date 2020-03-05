#ifndef CUBE_H
#define CUBE_H

#include "openglobject.h"

class Cube : public OpenGLObject
{
public:
    Cube();

    void create(const QVector3D &_pos, const QVector3D &_dim);
};

#endif // CUBE_H
