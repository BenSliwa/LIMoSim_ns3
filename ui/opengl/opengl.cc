#include "opengl.h"
#include <QFile>
#include <QDir>
#include <QTextStream>

namespace OpenGL
{

OpenGLContext::OpenGLContext()
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/


QStringList OpenGLContext::loadFile(const QString &_path)
{
    QStringList data;
    QFile file(_path);

    if(file.open(QIODevice::ReadOnly)) {
        QTextStream in(&file);

        while(!in.atEnd())
            data << in.readLine();

        file.close();
    }

    return data;
}


double OpenGLContext::map(float _value, float _inMin, float _inMax, float _outMin, float _outMax)
{
    return (_value-_inMin) * (_outMax-_outMin) / (_inMax-_inMin) + _outMin;
}

}
