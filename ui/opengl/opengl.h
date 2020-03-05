#ifndef OPENGL_OPENGL_H
#define OPENGL_OPENGL_H

#include "rect.h"
#include <QList>

namespace OpenGL
{

class OpenGLContext
{
public:
    OpenGLContext();


  //  virtual void drawPolyLine(const QList<Vertex> &_data) = 0;

    QStringList loadFile(const QString &_path);

    double map(float _value, float _inMin, float _inMax, float _outMin, float _outMax);
};

}

#endif // OPENGL_OPENGL_H
