#include "openglrenderer.h"
#include <iostream>
#include <math.h>
#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <QCoreApplication>
#include "earclipping.h"
#include <QTextStream>
#include <QMutex>
#include <cmath>

namespace OpenGL
{

static const char *vertexShaderSourceCore =
    "#version 150\n"
    "in vec4 vertex;\n"
    "in vec3 normal;\n"
    "in vec3 color;\n"
    "out vec3 vert;\n"
    "out vec3 vertNormal;\n"
    "out vec3 vertColor;\n"
    "uniform mat4 projMatrix;\n"
    "uniform mat4 mvMatrix;\n"
    "uniform mat3 normalMatrix;\n"
    "void main() {\n"
    "   vert = vertex.xyz;\n"
    "   vertNormal = normalMatrix * normal;\n"
    "   vertColor = color.xyz;\n"
    "   gl_Position = projMatrix * mvMatrix * vertex;\n"
    "}\n";


static const char *fragmentShaderSourceCore =
    "#version 150\n"
    "in highp vec3 vert;\n"
    "in highp vec3 vertNormal;\n"
    "in highp vec3 vertColor;\n"
    "out highp vec4 fragColor;\n"
    "uniform highp vec3 lightPos;\n"
    "void main() {\n"
    "   highp vec3 L = normalize(lightPos - vert);\n"
    "   highp float NL = max(dot(normalize(vertNormal), L), 0.0);\n"
    "   highp vec3 color = vertColor;\n"
    "   highp vec3 col = clamp(color * 0.2 + color * 0.8 * NL, 0.0, 1.0);\n"
    "   fragColor = vec4(col, 1.0);\n"
    "}\n";

static const char *vertexShaderSource =
    "attribute vec4 vertex;\n"
    "attribute vec3 normal;\n"
    "attribute vec3 color;\n"
    "varying vec3 vert;\n"
    "varying vec3 vertNormal;\n"
    "varying vec3 vertColor;\n"
    "uniform mat4 projMatrix;\n"
    "uniform mat4 mvMatrix;\n"
    "uniform mat3 normalMatrix;\n"
    "void main() {\n"
    "   vert = vertex.xyz;\n"
    "   vertNormal = normalMatrix * normal;\n"
    "   vertColor = color.xyz;\n"
    "   gl_Position = projMatrix * mvMatrix * vertex;\n"
    "}\n";

static const char *fragmentShaderSource =
    "varying highp vec3 vert;\n"
    "varying highp vec3 vertNormal;\n"
    "varying highp vec3 vertColor;\n"
    "uniform highp vec3 lightPos;\n"
    "void main() {\n"
    "   highp vec3 L = normalize(lightPos - vert);\n"
    "   highp float NL = max(dot(normalize(vertNormal), L), 0.0);\n"
    "   highp vec3 color = vertColor;\n"
    "   highp vec3 col = clamp(color * 0.2 + color * 0.8 * NL, 0.0, 1.0);\n"
    "   gl_FragColor = vec4(col, 1.0);\n"
    "}\n";

OpenGLRenderer::OpenGLRenderer(QObject *_parent) :
    QObject(_parent),
    m_program(0),
    m_fps(0),
    m_fpsCounter(0),
    m_renderingStatus(false)
{
    m_core = false;

    m_eye = QVector3D(1.18, 0.53, 2);
    m_elevation = 180;
    m_azimuth = 90;
    updateTarget();
    m_light = QVector3D(0, 0, 70);


    // UAV
    m_eye = QVector3D(0.945, -1.32, 1.58);
    m_target = QVector3D(0.945, -0.579, 0.912);
    m_elevation = 132;
    m_azimuth = 90;


    // ROAD
    m_eye = QVector3D(0.407, 0.201, 0.78);
    m_target = QVector3D(0.407, 0.202, -0.4);
    m_elevation = 180;
    m_azimuth = 90;

    // m_eye = QVector3D(0.407, 0.201, 0.4);

    connect(&m_fpsTimer, SIGNAL(timeout()), this, SLOT(onFpsTimeout()));
    m_fpsTimer.start(1000);

    m_background = "gray";
}

OpenGLRenderer::~OpenGLRenderer()
{
    delete m_program;
}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void OpenGLRenderer::setBackgroundColor(const QColor &_color)
{
    QVector3D color = computeColor(_color);
    glClearColor(color.x(), color.y(), color.z(), m_transparent ? 0 : 1);
}

QVector3D OpenGLRenderer::toNDC(const QVector3D &_vertex)
{
    float scale = 1000;
    float x = map(_vertex.x(), 0, scale, 0, 2);
    float y = map(_vertex.y(), 0, scale, 0, 2);
    float z = map(_vertex.z(), 0, scale, 0, 2);

    return QVector3D(x, y, z);
}

void OpenGLRenderer::drawPolygon(const OpenGLShape &_polygon, float _height, bool _static)
{
    EarClipping earClipping;
    QVector<QVector3D> vertices = _polygon.getVertices();
    QVector<QVector3D> earClipped = earClipping.triangulate(vertices);

    OpenGLObject object;
    object.setColor(_polygon.getColor());

    float scale = 1000;
    float height = map(_height, 0, scale, 0, 2) * 2;
    QVector3D h(0, 0, height);

    // draw the planes for ground and top
    for(int i=0; i<earClipped.size()/3; i++)
    {
        QVector3D v0 = earClipped.at(i*3); v0 = toNDC(v0);
        QVector3D v1 = earClipped.at(i*3+1); v1 = toNDC(v1);
        QVector3D v2 = earClipped.at(i*3+2); v2 = toNDC(v2);

        //
        QVector3D n(0,0,1);

        object.triangle(v0, v1, v2, n);
        object.triangle(v2, v1, v0, n);
        object.triangle(v1, v2, v0, n);

        //
        if(height>0)
        {
            object.triangle(v0+h, v1+h, v2+h, n);
            object.triangle(v2+h, v1+h, v0+h, n);
            object.triangle(v1+h, v2+h, v0+h, n);
        }
    }

    if(height>0)
    {
        // draw the side planes
        for(int i=0; i<vertices.size(); i++)
        {
            QVector3D v0 = vertices.at(i); v0 = toNDC(v0);
            QVector3D v1 = vertices.at((i+1)%vertices.size()); v1 = toNDC(v1);

            object.extrude(v0, v1, height);
            object.extrude(v1, v0, height);
        }

    }

    if(_static)
        m_staticBuffer.addObject(object);
    else
        m_dynamicBuffer.addObject(object);
//    return;
//    m_staticBuffer.addObject(object);
//    m_dynamicBuffer.addObject(object);
}

void OpenGLRenderer::draw3DTriangle(const OpenGLShape &_shape, float _height, bool _static)
{
    OpenGLObject object;
    object.setColor(_shape.getColor());

    QVector<QVector3D> vertices = _shape.getVertices();
    QVector3D v0 = vertices.at(0); v0 = toNDC(v0);
    QVector3D v1 = vertices.at(1); v1 = toNDC(v1);
    QVector3D v2 = vertices.at(2); v2 = toNDC(v2);

    float scale = 1000;
    float h = map(_height, 0, scale, 0, 2) * 2;

    object.triangle3d(v0, v1, v2, h);

    if (h > 0.0)
    {
        object.extrude(v0, v1, h);
        object.extrude(v1, v2, h);
        object.extrude(v2, v0, h);
    }

    if(_static)
        m_staticBuffer.addObject(object);
    else
        m_dynamicBuffer.addObject(object);

}

void OpenGLRenderer::drawCube(const OpenGLShape &_shape, GLfloat _height, bool _static)
{
    OpenGLObject object;
    object.setColor(_shape.getColor());

    QVector<QVector3D> vertices = _shape.getVertices();
    QVector3D v0 = vertices.at(0); v0 = toNDC(v0);
    QVector3D v1 = vertices.at(1); v1 = toNDC(v1);
    QVector3D v2 = vertices.at(2); v2 = toNDC(v2);
    QVector3D v3 = vertices.at(3); v3 = toNDC(v3);

    float scale = 1000;
    float z = v0.z();
    float h = map(_height, 0, scale, 0, 2) * 2;

    object.cube(v0, v1, v2, v3, h);
    if(h>0.0)
    {
        object.extrude(v0, v1, h);
        object.extrude(v1, v2, h);
        object.extrude(v2, v3, h);
        object.extrude(v3, v0, h);
    }

    if(_static)
        m_staticBuffer.addObject(object);
    else
        m_dynamicBuffer.addObject(object);
}

void OpenGLRenderer::drawCircle(QColor _color, QVector3D _center, GLfloat _radius, GLfloat _height, bool _static)
{
    OpenGLObject object;
    object.setColor(_color);

    QVector3D v0 = toNDC(_center);

    float radius = map(_radius, 0, 1000, 0, 2) * 2;
    float height = map(_height, 0, 1000, 0, 2) * 2
;    object.circle(v0, radius, height);

    if(_static)
        m_staticBuffer.addObject(object);
    else
        m_dynamicBuffer.addObject(object);
}

void OpenGLRenderer::initializeBuffer(const OpenGLObject &_buffer, QOpenGLVertexArrayObject *_vao, QOpenGLBuffer *_vbo)
{
    _vao->create();
    QOpenGLVertexArrayObject::Binder vaoBinder(_vao);

    QOpenGLBuffer vbo;
    _vbo->create();
    _vbo->bind();
    _vbo->allocate(_buffer.constData(), _buffer.count() * sizeof(GLfloat));
    setupVertexAttribs(_vbo);
}

void OpenGLRenderer::drawBuffer(const OpenGLObject &_buffer, QOpenGLVertexArrayObject *_vao)
{
    QOpenGLVertexArrayObject::Binder vaoBinder(_vao);

    m_program->bind();
    m_program->setUniformValue(m_projMatrixLoc, m_projection);
    m_program->setUniformValue(m_mvMatrixLoc, m_modelView);
    m_program->setUniformValue(m_lightPosLoc, m_light);

    QMatrix3x3 normalMatrix = m_world.normalMatrix();
    m_program->setUniformValue(m_normalMatrixLoc, normalMatrix);

    glDrawArrays(GL_TRIANGLES, 0, _buffer.vertexCount());
}

void OpenGLRenderer::drawDynamicBuffer(const OpenGLObject &_buffer)
{
    QOpenGLVertexArrayObject vao;
    QOpenGLBuffer vbo;
    initializeBuffer(_buffer, &vao, &vbo);
    drawBuffer(_buffer, &vao);

    // cleanup
    vbo.release();
    vbo.destroy();
    vao.release();
    vao.destroy();
}

void OpenGLRenderer::initializeGL()
{
    if(!m_renderingStatus)
        return;

    initializeOpenGLFunctions();
    setBackgroundColor(m_background);

    m_program = new QOpenGLShaderProgram;
    m_program->addShaderFromSourceCode(QOpenGLShader::Vertex, m_core ? vertexShaderSourceCore : vertexShaderSource);
    m_program->addShaderFromSourceCode(QOpenGLShader::Fragment, m_core ? fragmentShaderSourceCore : fragmentShaderSource);
    m_program->bindAttributeLocation("vertex", 0);
    m_program->bindAttributeLocation("normal", 1);
    m_program->bindAttributeLocation("color", 2);
    m_program->link();
    m_program->bind();

    m_projMatrixLoc = m_program->uniformLocation("projMatrix");
    m_mvMatrixLoc = m_program->uniformLocation("mvMatrix");
    m_normalMatrixLoc = m_program->uniformLocation("normalMatrix");
    m_lightPosLoc = m_program->uniformLocation("lightPos");

    initializeBuffer(m_staticBuffer, &m_vao, &m_staticVbo);

    m_program->release();
}

void OpenGLRenderer::paintGL()
{
    if(!m_renderingStatus)
        return;

    setBackgroundColor(m_background);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    m_projection = computeProjectionMatrix();
    m_modelView = computeModelViewMatrix();

    drawBuffer(m_staticBuffer, &m_vao);

    m_dynamicBuffer.lock();
    drawDynamicBuffer(m_dynamicBuffer);
    m_dynamicBuffer.clear();
    m_dynamicBuffer.unlock();

    m_program->release();
    m_fpsCounter++;
}

void OpenGLRenderer::setupVertexAttribs(QOpenGLBuffer *_vbo)
{
    _vbo->bind();
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    f->glEnableVertexAttribArray(0);
    f->glEnableVertexAttribArray(1);
    f->glEnableVertexAttribArray(2);

    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(GLfloat), 0);
    f->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(GLfloat), reinterpret_cast<void *>(3 * sizeof(GLfloat)));
    f->glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(GLfloat), reinterpret_cast<void *>(6 * sizeof(GLfloat)));
    _vbo->release();
}

void OpenGLRenderer::resizeGL(int w, int h)
{
    m_projection.setToIdentity();
    m_projection.perspective(45.0f, GLfloat(w) / h, 0.01f, 100.0f);
}

void OpenGLRenderer::zoomIn()
{
    QVector3D dir = (m_target - m_eye).normalized() * 0.1;
    m_eye = m_eye + dir;
    m_target = m_target + dir;
}

void OpenGLRenderer::zoomOut()
{
    QVector3D dir = (m_target - m_eye).normalized() * 0.1;
    m_eye = m_eye - dir;
    m_target = m_target - dir;
}

void OpenGLRenderer::handleMouseMove(float _x, float _y)
{
    float xRange = m_viewportSize.width();
    float yRange = m_viewportSize.height();
    float x = map(_x, -xRange, xRange, -1, 1);
    float y = map(_y, -yRange, yRange, -1, 1);

    QVector3D inc = QVector3D(-x, -y, 0);
    m_eye += inc;
    m_target += inc;
}


QString OpenGLRenderer::info()
{
    QString data;
    QTextStream stream(&data);
    stream.setRealNumberPrecision(3);
    stream << "eye: " << m_eye.x() << "/" << m_eye.y() << "/" << m_eye.z() << "  ";
    stream << "ele: " << m_elevation << " " << "az: " << m_azimuth << " ";
    stream << "target: " << m_target.x() << "/" << m_target.y() << "/" << m_target.z() << "  ";
    stream << "fps: " << m_fps;

    return data;
}

void OpenGLRenderer::setRenderingStatus(bool _ready)
{
    m_renderingStatus = _ready;

    qDebug() << "static vertices: " << m_staticBuffer.vertexCount();
}

void OpenGLRenderer::updateEye(const QVector3D &_increment)
{
    m_eye += _increment;
}

void OpenGLRenderer::updateTarget(double _elevation, double _azimuth)
{
    m_elevation += _elevation;
    m_azimuth += _azimuth;

    updateTarget();
}

void OpenGLRenderer::updateTarget()
{
    m_target = m_eye + fromSphere(1, m_elevation, m_azimuth);
}

QVector3D OpenGLRenderer::fromSphere(float _r, float _theta, float _phi)
{
    float theta = _theta * 3.1415 / 180;
    float phi = _phi * 3.1415 / 180;

    float x = _r * sin(theta) * cos(phi);
    float y = _r * sin(theta) * sin(phi);
    float z =  _r * cos(theta);

    return QVector3D(x, y, z);
}

QMatrix4x4 OpenGLRenderer::computeProjectionMatrix()
{
    return m_projection;
}

QMatrix4x4 OpenGLRenderer::computeModelViewMatrix()
{
    QMatrix4x4 mv;
    mv.setToIdentity();
    mv.lookAt(m_eye, m_target, QVector3D(0, 1, 0));
    return mv;
}

void OpenGLRenderer::setEye(const QVector3D &_eye)
{
    m_eye = _eye;
}

void OpenGLRenderer::setElevation(float _elevation)
{
    m_elevation = _elevation;
}

void OpenGLRenderer::setAzimuth(float _azimuth)
{
    m_azimuth = _azimuth;
}

QVector3D OpenGLRenderer::getEye()
{
    return m_eye;
}

QVector3D OpenGLRenderer::getTarget()
{
    return m_target;
}

/*************************************
 *           PRIVATE METHODS         *
 ************************************/


double OpenGLRenderer::map(float _value, float _inMin, float _inMax, float _outMin, float _outMax)
{
    return (_value-_inMin) * (_outMax-_outMin) / (_inMax-_inMin) + _outMin;
}

QColor OpenGLRenderer::computeColor(const QVector3D &_vertex, const QVector3D &_normal, const QVector3D &_color)
{
    QVector3D light = m_projection * m_modelView * m_light;
    QVector3D L = (light - _vertex);


    #ifdef Q_OS_ANDROID
        return QColor("white"); // this is never called
    #else
        float NL = std::fmax(QVector3D::dotProduct(_normal.normalized(), L), 0.0);
        QVector3D c = clamp(_color * 0.2 + _color * 0.8 * NL, 0.0, 1.0);

        return QColor::fromRgbF(c.x(), c.y(), c.z());
    #endif


}

QVector3D OpenGLRenderer::computeColor(const QColor &_color)
{
    qreal r,g,b;
    _color.getRgbF(&r,&g,&b);
    return QVector3D(r, g, b);
}

QVector3D OpenGLRenderer::clamp(const QVector3D &_vector, float _low, float _high)
{
    return QVector3D(clamp(_vector.x(), _low, _high), clamp(_vector.y(), _low, _high), clamp(_vector.z(), _low, _high));
}

float OpenGLRenderer::clamp(float _value, float _low, float _high)
{
    float value = _value;
    if(value<_low)
        value = _low;
    if(value>_high)
        value = _high;

    return value;
}

/***************************************
 *                SLOTS                *
 **************************************/

void OpenGLRenderer::paint()
{
    if(!m_program)
    {
        initializeGL();
        resizeGL(m_viewportSize.width(), m_viewportSize.height());
    }
    paintGL();


}

void OpenGLRenderer::onFpsTimeout()
{
    m_fps = m_fpsCounter;
    m_fpsCounter = 0;
}


}
