#ifndef LIMOSIM_UIMANAGER_H
#define LIMOSIM_UIMANAGER_H

#include <QObject>
#include <QDebug>
#include <QQmlApplicationEngine>
#include <QQuickView>

#include "ui/opengl/openglcanvas.h"
#include "ui/opengl/openglrenderer.h"
#include "ui/opengl/openglwindow.h"
#include "ui/opengl/openglshape.h"
#include "LIMoSim/world/vector3d.h"



using namespace OpenGL;

namespace LIMoSim
{
class Visualizer;

class UiManager : public QObject
{
    Q_OBJECT
public:
    UiManager(QObject *_parent = 0);

    void loadQml();

    //
    void addVisualizer(Visualizer *_visualizer);

    // Visualization
    void drawNodes(Canvas *_canvas);
    void drawSegments(Canvas *_canvas);
    void drawBuildings(Canvas *_canvas);
    void drawVehicles(Canvas *_canvas);

    // Events
    Q_INVOKABLE void handleExternalDrop(const QStringList &_files);
    Q_INVOKABLE void handleKeyPress(int _key);
    Q_INVOKABLE void handleScaleChange(double _scale);

    //
    void updateCamera(const Vector3d &_tracked);
    void setTrackedVehicle(const QString &_id);

    //
    OpenGL::OpenGLCanvas* getOpenGLCanvas();
    OpenGL::OpenGLRenderer* getRenderer();

    static double  getScale();


private slots:
    void onInitialized();
    void onPaint();

signals:
    void info(const QString &_info);

private:
    QQuickView m_qml;
    OpenGL::OpenGLWindow *p_window;
    OpenGL::OpenGLRenderer *p_renderer;
    OpenGL::OpenGLCanvas m_canvas;

    QString m_trackedVehicle="T0";
    bool m_trackVehicle = false;
    QVector<Visualizer*> m_visualizer;
    static double s_scale;
};

}

#endif // UIMANAGER_H
