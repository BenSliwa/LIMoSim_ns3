#ifndef OPENGL_OPENGLITEM_H
#define OPENGL_OPENGLITEM_H

#include <QtQuick/QQuickItem>

#include <QtQuick/qquickwindow.h>
#include <QTimer>
#include <QMutex>

// multiple polygons:
// https://stackoverflow.com/questions/26944959/opengl-separating-polygons-inside-vbo
// http://programming4.us/multimedia/8302.aspx

#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QMatrix4x4>
#include "openglobject.h"
#include "openglshape.h"

QT_FORWARD_DECLARE_CLASS(QOpenGLShaderProgram)

namespace OpenGL
{

class OpenGLRenderer : public QObject, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    OpenGLRenderer(QObject *_parent = 0);
    ~OpenGLRenderer();

    void setViewportSize(const QSize &size) { m_viewportSize = size; }
    void setBackgroundColor(const QColor &_color);


    QVector3D toNDC(const QVector3D &_vertex);
    void drawPolygon(const OpenGLShape &_polygon, float _height, bool _static=true);
    void draw3DTriangle(const OpenGLShape &_shape, float _height, bool _static=true);
    void drawCube(const OpenGLShape &_shape, GLfloat _height, bool _static=true);
    void drawCircle(QColor _color, QVector3D _center, GLfloat _radius, GLfloat _height = 0, bool _static=true);
    void initializeBuffer(const OpenGLObject &_buffer, QOpenGLVertexArrayObject *_vao, QOpenGLBuffer *_vbo);
    void drawBuffer(const OpenGLObject &_buffer, QOpenGLVertexArrayObject *_vao);
    void drawDynamicBuffer(const OpenGLObject &_buffer);

    void initializeGL();
    void paintGL();
    void setupVertexAttribs(QOpenGLBuffer *_vbo);

    void resizeGL(int width, int height);
    void zoomIn();
    void zoomOut();
    void handleMouseMove(float _x, float _y);

    QString info();
    void setRenderingStatus(bool _ready);

    void updateEye(const QVector3D &_increment);
    void updateTarget(double _elevation, double _azimuth);
    void updateTarget();
    QVector3D fromSphere(float _r, float _theta, float _phi);
    QMatrix4x4 computeProjectionMatrix();
    QMatrix4x4 computeModelViewMatrix();

    //
    void setEye(const QVector3D &_eye);
    void setElevation(float _elevation);
    void setAzimuth(float _azimuth);
    QVector3D getEye();
    QVector3D getTarget();


public:
    double map(float _value, float _inMin, float _inMax, float _outMin, float _outMax);
    QColor computeColor(const QVector3D &_vertex, const QVector3D &_normal, const QVector3D &_color);
    QVector3D computeColor(const QColor &_color);
    QVector3D clamp(const QVector3D &_vector, float _low, float _high);
    float clamp(float _value, float _low, float _high);


public slots:
    void paint();
    void onFpsTimeout();

private:
    QVector3D m_eye;
    float m_elevation;
    float m_azimuth;
    QVector3D m_target;
    QVector3D m_light;


    QSize m_viewportSize;


    bool m_core;
    bool m_transparent;

    QOpenGLVertexArrayObject m_vao;
    QOpenGLBuffer m_staticVbo;
    QOpenGLShaderProgram *m_program;
    OpenGLObject m_staticBuffer;
    OpenGLObject m_dynamicBuffer;
    int m_projMatrixLoc;
    int m_mvMatrixLoc;
    int m_normalMatrixLoc;
    int m_lightPosLoc;

    QMatrix4x4 m_projection;
    QMatrix4x4 m_world;
    QMatrix4x4 m_modelView;

    QTimer m_fpsTimer;
    int m_fps;
    int m_fpsCounter;

    bool m_renderingStatus;
    QColor m_background;

};

}

#endif // OPENGL_OPENGLITEM_H
