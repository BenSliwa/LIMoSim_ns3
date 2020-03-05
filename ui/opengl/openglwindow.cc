#include "openglwindow.h"

#include <QtGui/QOpenGLShaderProgram>
#include <QtGui/QOpenGLContext>
#include <QDebug>
#include "ui/uimanager.h"

#include "LIMoSim/simulation/simulation.h"
#include "ui/standalone/standaloneeventscheduler.h"

namespace OpenGL
{


OpenGLWindow::OpenGLWindow()
    : QQuickItem(),
      m_t(0),
      m_renderer(0)
{
    setAcceptedMouseButtons(Qt::AllButtons);

    connect(this, &QQuickItem::windowChanged, this, &OpenGLWindow::handleWindowChanged);
    connect(&m_timer, SIGNAL(timeout()), this, SLOT(onTimeout()));

    m_timer.start(1000.0 / 60.0);
}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void OpenGLWindow::setT(qreal t)
{
    if (t == m_t)
        return;
    m_t = t;

    emit tChanged();
    if (window())
        window()->update();
}

OpenGLRenderer* OpenGLWindow::getRenderer()
{
    return m_renderer;
}

int OpenGLWindow::randomInt(int _min, int _max)
{
    int number = qrand() % ((_max + 1) - _min) + _min;
    return number;
}

void OpenGLWindow::handleScale(float _scale, double _x, double _y)
{
    if(_scale>0)
        m_renderer->zoomIn();
    else
        m_renderer->zoomOut();
}

void OpenGLWindow::mousePressEvent(QMouseEvent *_event)
{
    QPointF screenPoint(_event->pos().x(), height() - _event->pos().y());
    m_lastPosition = _event->pos();

    emit mousePressed(screenPoint);
}

void OpenGLWindow::mouseReleaseEvent(QMouseEvent *_event)
{
    QPointF screenPoint(_event->pos().x(), height() - _event->pos().y());
    emit mouseReleased(screenPoint);
}

void OpenGLWindow::mouseMoveEvent(QMouseEvent *_event)
{
    QPointF position = _event->pos();
    QPointF screenPoint(_event->pos().x(), height() - _event->pos().y());
    QPointF offset = position - m_lastPosition;

    m_lastPosition = position;

    handleMouseMove(offset);
}


void OpenGLWindow::handleMousePress()
{

}

void OpenGLWindow::handleMouseRelease()
{

}

void OpenGLWindow::handleMouseMove(const QPointF &_delta)
{
    m_renderer->handleMouseMove(_delta.x(), -_delta.y());
}

void OpenGLWindow::handleWindowChanged(QQuickWindow *win)
{
    if (win)
    {
        connect(win, &QQuickWindow::beforeSynchronizing, this, &OpenGLWindow::sync, Qt::DirectConnection);
        connect(win, &QQuickWindow::sceneGraphInvalidated, this, &OpenGLWindow::cleanup, Qt::DirectConnection);

        win->setClearBeforeRendering(false);
    }
}

void OpenGLWindow::cleanup()
{
    if (m_renderer) {
        delete m_renderer;
        m_renderer = 0;
    }
}

void OpenGLWindow::onTimeout()
{
    if (window())
        window()->update();
}

void OpenGLWindow::sync()
{
    if (!m_renderer) {
        m_renderer = new OpenGL::OpenGLRenderer();
        connect(window(), &QQuickWindow::beforeRendering, m_renderer, &OpenGL::OpenGLRenderer::paint, Qt::DirectConnection);

        emit initialized();


    }
    m_renderer->setViewportSize(window()->size() * window()->devicePixelRatio());

    emit updated();

    //
    LIMoSim::Simulation *simulation = LIMoSim::Simulation::getInstance(0);
    LIMoSim::StandaloneEventScheduler *scheduler = dynamic_cast<LIMoSim::StandaloneEventScheduler*>(simulation->getEventScheduler());
    if(scheduler)
        scheduler->handleNextEvent();
}

}
