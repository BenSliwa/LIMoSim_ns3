#include "openglobject.h"

#include <math.h>

namespace OpenGL
{

OpenGLObject::OpenGLObject() :
    m_count(0),
    m_color("white")
{

}
const GLfloat* OpenGLObject::constData() const
{
    return m_data.constData();
}

int OpenGLObject::count() const
{
    return m_count;
}

int OpenGLObject::vertexCount() const
{
    return m_count / 9;
}

void OpenGLObject::add(const QVector3D &v, const QVector3D &n)
{
    int r, g, b;
    m_color.getRgb(&r,&g,&b);

    m_data << v.x() << v.y() << v.z();
    m_data << n.x() << n.y() << n.z();
    m_data << r/255.0 << g/255.0 << b/255.0;

    m_count += 9;
}

void OpenGLObject::triangle(const QVector3D &_v0, const QVector3D &_v1, const QVector3D &_v2, const QVector3D &_n)
{
    add(_v0, _n);
    add(_v1, _n);
    add(_v2, _n);
}

void OpenGLObject::triangle3d(const QVector3D &_v0, const QVector3D &_v1, const QVector3D &_v2, float _height)
{
    // draw upper triangle
    QVector3D n = QVector3D::normal(QVector3D(_v2.x() - _v0.x(), _v2.y() - _v0.y(), 0.0f), QVector3D(_v1.x() - _v0.x(), _v1.y() - _v0.y(), 0.0f));
    triangle(_v0, _v2, _v1, n);

    // draw lower triangle
    n = QVector3D::normal(QVector3D(_v0.x() - _v2.x(), _v0.y() - _v2.y(), 0.0f), QVector3D(_v1.x() - _v2.x(), _v1.y() - _v2.y(), 0.0f));
    QVector3D h = QVector3D(0, 0, _height);
    triangle(_v2+h, _v0+h, _v1+h, n);
}

void OpenGLObject::cube(const QVector3D &_v0, const QVector3D &_v1, const QVector3D &_v2, const QVector3D &_v3, float _height)
{
    // draw upper quad
    QVector3D n = QVector3D::normal(QVector3D(_v3.x() - _v0.x(), _v3.y() - _v0.y(), 0.0f), QVector3D(_v1.x() - _v0.x(), _v1.y() - _v0.y(), 0.0f));
    triangle(_v0, _v3, _v1, n);
    triangle(_v2, _v1, _v3, n);

    // draw lower quad
    n = QVector3D::normal(QVector3D(_v0.x() - _v3.x(), _v0.y() - _v3.y(), 0.0f), QVector3D(_v1.x() - _v3.x(), _v1.y() - _v3.y(), 0.0f));
    QVector3D h = QVector3D(0, 0, _height);
    triangle(_v3+h, _v0+h, _v1+h, n);
    triangle(_v1+h, _v2+h, _v3+h, n);
}

void OpenGLObject::circle(const QVector3D &c, const float _radius, const float _height)
{
    // Mapping bounds for num of segments and linewidth
    /*********************************************************
     *      radius      num_segments    linewidth
     * max   100            100             0.1
     * min   5              20              0.5
     * *******************************************************/
    // purposedly inversed bounds for linewidth for inverse proportionality

    int num_segments = (int) map(_radius, 0.5, 100, 10, 100);
    double linewidth = map(_radius, 0.5, 100, 1.5, 0.1);

    QVector3D v, v1;
    for(int ii = 0; ii < num_segments; ii++)
    {
        float theta = 2.0f * 3.1415926f * float(ii) / float(num_segments);//get the current angle
        float theta1 = 2.0f * 3.1415926f * float(ii+1) / float(num_segments);//get the next angle

        float x = _radius * cosf(theta);//calculate the x component
        float y = _radius * sinf(theta);//calculate the y component
        float x1 = _radius * cosf(theta1);//calculate the next x component
        float y1 = _radius * sinf(theta1);//calculate the next y component
        v = c + QVector3D (x,y,0);
        v1 = c + QVector3D (x1,y1,0);

        QVector3D dir = v1 - v;
        QVector3D leftDir = QVector3D(-dir.y(), dir.x(),0);
        QVector3D rightDir = QVector3D(dir.y(), -dir.x(),0);
        leftDir.normalize();
        cube(v,
             v + rightDir * linewidth/2,
             v1 + rightDir * linewidth/2,
             v1,
             _height);

    }
}

void OpenGLObject::extrude(const QVector3D &_v0, const QVector3D &_v1, float _height)
{
    QVector3D n = QVector3D::normal(QVector3D(0.0f, 0.0f, -0.1f), QVector3D(_v1.x() - _v0.x(), _v1.y() - _v0.y(), 0.0f));
    QVector3D h = QVector3D(0, 0, _height);

    triangle(_v0 + h, _v0, _v1 + h, n);
    triangle(_v1, _v1+h, _v0, n);
}

void OpenGLObject::addObject(const OpenGLObject &_object)
{
    m_mutex.lock();

    m_data << _object.getData();
    m_count += _object.count();

    m_mutex.unlock();
}

void OpenGLObject::setColor(const QColor &_color)
{
    m_color = _color;
}

QColor OpenGLObject::getColor() const
{
    return m_color;
}

const QVector<GLfloat>& OpenGLObject::getData() const
{
    return m_data;
}

void OpenGLObject::lock()
{
    m_mutex.lock();
}

void OpenGLObject::unlock()
{
    m_mutex.unlock();
}

void OpenGLObject::clear()
{
    m_data.clear();
    m_count = 0;
}

double OpenGLObject::map(float _value, float _inMin, float _inMax, float _outMin, float _outMax)
{
    return (_value-_inMin) * (_outMax-_outMin) / (_inMax-_inMin) + _outMin;
}

}
