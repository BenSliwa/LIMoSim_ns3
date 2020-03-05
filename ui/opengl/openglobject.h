#ifndef OPENGLOBJECT_H
#define OPENGLOBJECT_H

#include <QDebug>
#include <qopengl.h>
#include <QVector>
#include <QVector3D>
#include <QColor>
#include <QMutex>

namespace OpenGL
{

class OpenGLObject
{
public:
    OpenGLObject();

    const GLfloat* constData() const;
    int count() const;
    int vertexCount() const;

    void add(const QVector3D &v, const QVector3D &n);
    void triangle(const QVector3D &_v0, const QVector3D &_v1, const QVector3D &_v2, const QVector3D &_n);
    void triangle3d(const QVector3D &_v0, const QVector3D &_v1, const QVector3D &_v2, float _height);
    void cube(const QVector3D &_v0, const QVector3D &_v1, const QVector3D &_v2, const QVector3D &_v3, float _height);
    void circle(const QVector3D &c, const float _radius, const float _height = 0);

    void extrude(const QVector3D &_v0, const QVector3D &_v1, float _height);

    void addObject(const OpenGLObject &_object);

    //
    void setColor(const QColor &_color);
    QColor getColor() const;
    const QVector<GLfloat>& getData() const;

    void lock();
    void unlock();
    void clear();

    double map(float _value, float _inMin, float _inMax, float _outMin, float _outMax);

protected:
    QVector<GLfloat> m_data;
    int m_count;
    QColor m_color;

    QMutex m_mutex;
};


}

#endif // OPENGLOBJECT_H
