#ifndef OPENGL_OPENGLWINDOW_H
#define OPENGL_OPENGLWINDOW_H

#include <QtQuick/QQuickItem>
#include <QtGui/QOpenGLShaderProgram>
#include <QtGui/QOpenGLFunctions>
#include <QtGui/QOpenGLFunctions_4_0_Core>
#include <QMouseEvent>
#include "openglrenderer.h"


namespace OpenGL
{

class OpenGLWindow : public QQuickItem
{
    Q_OBJECT
    Q_PROPERTY(qreal t READ t WRITE setT NOTIFY tChanged)

public:
    OpenGLWindow();

    qreal t() const { return m_t; }
    void setT(qreal t);

    OpenGLRenderer* getRenderer();

    int randomInt(int _min, int _max);

    Q_INVOKABLE void handleScale(float _scale, double _x, double _y);

    void mousePressEvent(QMouseEvent *_event);
    void mouseReleaseEvent(QMouseEvent *_event);
    void mouseMoveEvent(QMouseEvent *_event);

    void handleMousePress();
    void handleMouseRelease();
    void handleMouseMove(const QPointF &_delta);

signals:
    void tChanged();
    void updated();
    void initialized();

    void mousePressed(const QPointF &_position);
    void mouseReleased(const QPointF &_position);
    void mouseMoved(const QPointF &_position, const QPointF &_delta);

    void zoomChanged(double _scale);
    void offsetChanged(const QPointF &_offset);

public slots:
    void onTimeout();
    void sync();
    void cleanup();


private slots:
    void handleWindowChanged(QQuickWindow *win);

private:
    QTimer m_timer;
    qreal m_t;
    OpenGLRenderer *m_renderer;

    QPointF m_lastPosition;
};

}

#endif // OPENGL_OPENGLWINDOW_H
