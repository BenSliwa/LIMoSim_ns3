#include "uimanager.h"
#include <QQmlContext>
#include <Qt3DCore/QTransform>
#include "LIMoSim/world/world.h"
#include "LIMoSim/world/vehiclemanager.h"
#include "LIMoSim/world/road/lanesegment.h"
#include "ui/export/epsdocument.h"
#include "ui/export/epscanvas.h"
#include "LIMoSim/mobility/car/strategic/strategicmodel.h"
#include "LIMoSim/settings/osm/wgs84.h"
#include "visualizer.h"
#include "ui/opengl/openglshape.h"
#include "ui/export/raytracingheatmap.h"
#include "LIMoSim/world/raytracing/antenna.h"
#include "ui/data/uidatamanager.h"
#include "ui/data/uidatadrawer.h"
#include "ui/uimanagerservice.h"

#include "demo/deliverylistservice.h"
#include <math.h>

namespace LIMoSim
{

double UiManager::s_scale = 1;

UiManager::UiManager(QObject *_parent) :
    QObject(_parent),
    p_window(0),
    p_renderer(0)
{
    ui::UiManagerService::getInstance()->setUiManager(this);
}


/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void UiManager::loadQml()
{
    // load the qml file and register Qt classes
    QQmlContext *context = m_qml.rootContext();
    context->setContextProperty("UiManager", this);
    qmlRegisterType<OpenGL::OpenGLWindow>("OpenGL", 1, 0, "OpenGLWindow");
    m_qml.setSource(QUrl(QLatin1String("qrc:/main.qml")));
    m_qml.show();

    //
    QObject *object = m_qml.rootObject()->findChild<OpenGLWindow*>("map");
    p_window = qobject_cast<OpenGLWindow*>(object);
    connect(p_window, SIGNAL(initialized()), this, SLOT(onInitialized()));

    #ifdef Q_OS_ANDROID
        onInitialized();
    #else

    #endif
}

/*************************************
 *            VISUALIZATION          *
 ************************************/

void UiManager::addVisualizer(Visualizer *_visualizer)
{
    m_visualizer << _visualizer;
}

void UiManager::drawNodes(Canvas *_canvas)
{
    World *world = World::getInstance();
    std::map<std::string, Node*> nodes = world->getNodes();
    std::map<std::string, Node*>::iterator it;
    for(it=nodes.begin(); it!=nodes.end(); it++)
    {
        Node *node = it->second;
        if(world->filterRoads(node->getWays()).size()==0) // do not draw nodes for buildings
            continue;

        _canvas->drawNode(node);
    }
}

void UiManager::drawSegments(Canvas *_canvas)
{
    World *world = World::getInstance();
    std::map<std::string, Road*> roads = world->getRoads();
    std::map<std::string, Road*>::iterator it;
    for(it=roads.begin(); it!=roads.end(); it++)
    {
        Road *road = it->second;
        std::vector<RoadSegment*> segments = road->getSegments();

        for(unsigned int i=0; i<segments.size(); i++)
        {
            RoadSegment *segment = segments.at(i);
            _canvas->drawRoadSegment(segment);
        }
    }
}

void UiManager::drawBuildings(Canvas *_canvas)
{
    World *world = World::getInstance();
    std::map<std::string, Building*> buildings = world->getBuildings();
    std::map<std::string, Building*>::iterator it;
    for(it=buildings.begin(); it!=buildings.end(); it++)
    {
        Building *building = it->second;
        if (delivery::DeliveryListService::getInstance()->
                getDeliveryStatusColor(building->getId()) == "darkgray") {
            _canvas->drawBuilding(building);
        }
//        _canvas->drawBuilding(building);
    }
}


QVector<QVector3D> trajectory;
void UiManager::drawVehicles(Canvas *_canvas)
{
//    VehicleManager *vehicleManager = VehicleManager::getInstance();
//    std::map<std::string, Vehicle*> vehicles = vehicleManager->getVehicles();
//    std::map<std::string, Vehicle*>::iterator it;

    //auto vehicleUIData = UI::Data::UIDataManager::getInstance()->getVehiclesData();

    UI::Data::UIDataDrawer drawer(&m_canvas, p_renderer);
    if (m_trackVehicle) {
        Vehicle* trackedVehicle = VehicleManager::getInstance()
                ->getVehicle(m_trackedVehicle.toStdString());
        if(trackedVehicle){
            updateCamera(trackedVehicle->getPosition());
        }
    }
    drawer.drawVehicles();
    drawer.drawStaticNodes();
    drawer.drawConnectionStatuses();
    drawer.drawConnectionLinks();

//    for(it=vehicles.begin(); it!=vehicles.end(); it++)
//    {
//        Vehicle *vehicle = it->second;
//        Vector3d p = vehicleUIData.at(it->first)->getPosition();
//        float x = p.x;
//        float y = p.y;
//        float z = p.z;
//        float w = 3;

//        if(vehicle->getType()=="Car")
//        {
//            Car *car = dynamic_cast<Car*>(vehicle);

//            float w = car->getWidth();
//            float l = car->getLength();
//            Vector3d left = Vector3d::fromSphere(90, 90 + car->getOrientation().z, w/2);
//            Vector3d back = left.rotateRight().normed() * l;

//            OpenGLShape shape("yellow");
//            shape.addVertex(m_canvas.fromVector(p + left));
//            shape.addVertex(m_canvas.fromVector(p - left));
//            shape.addVertex(m_canvas.fromVector(p - left + back));
//            shape.addVertex(m_canvas.fromVector(p + left + back));

//            p_renderer->drawCube(shape, 1.5, false);


//            if(car->getId()==m_trackedVehicle.toStdString())
//                updateCamera(p);

//            OpenGLShape beam("red");
//            float bW = 4;
//            float bL = 40;
//            Vector3d fDir = Vector3d(0,1,0) * bL;
//            Vector3d lDir = Vector3d(-1,0,0) * bW;

//            std::vector<Vector3d> b;
//            b.push_back(p + 0.5*lDir);
//            b.push_back(p -0.5*lDir);
//            b.push_back(p -2*lDir + fDir);
//            b.push_back(p +2*lDir + fDir);

//            float alpha = Vector3d::toRad(car->getOrientation().z-90);
//            for(int i=0; i<b.size(); i++)
//            {
//                Vector3d v0 = vehicle->getPosition();
//                Vector3d v1 = b.at(i);
//                Vector3d v2 = v1-v0;

//                Vector3d r;
//                r.x = cos(alpha)*v2.x - sin(alpha)*v2.y;
//                r.y = sin(alpha)*v2.x + cos(alpha)*v2.y;
//                r.z = 0;

//                beam.addVertex(m_canvas.fromVector(r+v0));
//            }
//            p_renderer->drawCube(beam, 0.5, false);

//        }
//        else if (vehicle->getType() == "UAV")
//        {
//            UI::Data::UIDataDrawer drawer(&m_canvas, p_renderer);
//            drawer.drawVehicles();

////            OpenGLShape shape("yellow");
////            if (vehicle->getId() =="U1") {
////                shape.setColor("blue");
////            } else if (vehicle->getId() == "U2") {
////                shape.setColor("red");
////            }

////            Orientation3d orientation = vehicle->getOrientation();
////            Qt3DCore::QTransform orientationTransformation;
////            orientationTransformation.setRotation(Qt3DCore::QTransform::fromEulerAngles(orientation.roll(), 90 - orientation.pitch(), orientation.yaw()));
////            orientationTransformation.setTranslation(QVector3D(p.x, p.y, p.z));
////            shape.addVertex(orientationTransformation.translation() + (orientationTransformation.rotation() * QVector3D(w, 0, 0)));
////            shape.addVertex(orientationTransformation.translation() + (orientationTransformation.rotation() * QVector3D(-w, w/2, 0)));
////            shape.addVertex(orientationTransformation.translation() + (orientationTransformation.rotation() * QVector3D(-w, -w/2, 0)));

////            p_renderer->draw3DTriangle(shape, w, false);
//        }
//        else
//        {
//            OpenGLShape shape("yellow");
//            shape.addVertex(QVector3D(x-w, y-w, z));
//            shape.addVertex(QVector3D(x-w, y+w, z));
//            shape.addVertex(QVector3D(x+w, y+w, z));
//            shape.addVertex(QVector3D(x+w, y-w, z));

//            p_renderer->drawCube(shape, w, false);


//            //
//            trajectory << QVector3D(x, y, z);
//            if(trajectory.size()>200)
//                trajectory.pop_front();


//            for(int i=0; i<trajectory.size(); i++)
//            {
//                OpenGLShape t("red");
//                QVector3D v0 = trajectory.at(i);

//                float x = v0.x();
//                float y = v0.y();
//                float z = v0.z();
//                float w = 1;

//                t.addVertex(QVector3D(x-w, y-w, z));
//                t.addVertex(QVector3D(x-w, y+w, z));
//                t.addVertex(QVector3D(x+w, y+w, z));
//                t.addVertex(QVector3D(x+w, y-w, z));

//                p_renderer->drawCube(t, w, false);
//            }

//        }

//    }
}

/*************************************
 *               EVENTS              *
 ************************************/

void UiManager::handleExternalDrop(const QStringList &_files)
{
    qDebug() << "UiManager::handleExternalDrop" << _files;

    // TODO:
}

void UiManager::handleKeyPress(int _key)
{
    qDebug() << "UiManager::handleKeyPress" << _key;

    // TODO:

    if(_key==Qt::Key_P)
    {
        EpsCanvas canvas;
        canvas.setOpenGLRenderer(p_renderer);
        canvas.init();
        drawBuildings(&canvas);
        drawSegments(&canvas);

        canvas.save("screen.eps");
    }
    else if(_key==Qt::Key_R)
    {
        RayTracingHeatMap heatMap;

        World *world = World::getInstance();
        Vector3d min = Vector3d(world->getBoxMin().x, world->getBoxMin().y, 20);
        Vector3d max = Vector3d(world->getBoxMax().x, world->getBoxMax().y, 20);

        Vector3d tx = (min + max) / 2;
        tx.z = 25;

        Antenna antenna(tx, 0);


        heatMap.save(antenna, 0, "heatmap_0.eps");
        heatMap.save(antenna, 5, "heatmap_5.eps");
        heatMap.save(antenna, 10, "heatmap_10.eps");
        heatMap.save(antenna, 15, "heatmap_15.eps");
        heatMap.save(antenna, 20, "heatmap_20.eps");
        heatMap.save(antenna, 25, "heatmap_25.eps");
        heatMap.save(antenna, 30, "heatmap_30.eps");
        heatMap.save(antenna, 35, "heatmap_35.eps");
        heatMap.save(antenna, 40, "heatmap_40.eps");
        heatMap.save(antenna, 45, "heatmap_45.eps");
        heatMap.save(antenna, 50, "heatmap_50.eps");
    }
    else if(_key==Qt::Key_Plus)
    {
        p_renderer->zoomIn();
    }
    else if(_key==Qt::Key_Minus)
    {
        p_renderer->zoomOut();
    }
    else if(_key==Qt::Key_Left)
    {
        p_renderer->updateTarget(0, 1);
    }
    else if(_key==Qt::Key_Right)
    {
        p_renderer->updateTarget(0, -1);
    }
    else if(_key==Qt::Key_Up)
    {
        // p_renderer->updateEye(QVector3D(0, 0.1, 0));
        p_renderer->updateTarget(1, 0);
    }
    else if(_key==Qt::Key_Down)
    {
        p_renderer->updateTarget(-1, 0);
    }
    else if(_key==Qt::Key_Space)
    {
        p_renderer->setEye(QVector3D(1.18, 0.53, 2));
        p_renderer->setElevation(180);
        p_renderer->setAzimuth(90);
        p_renderer->updateTarget();
    }
    else if(_key==Qt::Key_W)
    {
        // car surrogate
        VehicleManager *vm = VehicleManager::getInstance();
        Car *car = dynamic_cast<Car*>(vm->getVehicle("C0"));
        if(car)
        {
            float tau_s = 1.0;
            float step_s = 0.25;
            int steps = tau_s/step_s;

            for(int i=0; i<steps; i++)
                car->move(step_s);
        }
    }
    else if(_key==Qt::Key_S)
    {
        //                  TODO: this is a dirty hack, add something better
        for(int i=0; i<m_visualizer.size(); i++)
            m_visualizer.at(i)->update(0);
    }
    else if(_key==Qt::Key_D)
    {

    }
    else if(_key==Qt::Key_A)
    {

    }
    else if (_key==Qt::Key_T) {
        auto vehicles = VehicleManager::getInstance()->getVehicles();
        if (!m_trackVehicle) {
            m_trackedVehicle = QString::fromStdString(vehicles.begin()->first);
            m_trackVehicle = true;
        } else {

            auto it = vehicles.find(m_trackedVehicle.toStdString());
            if (it == --vehicles.end()) {
                m_trackVehicle = false;
            } else {
                m_trackedVehicle = QString::fromStdString((++it)->first);
            }
        }

        if (!m_trackVehicle) {
             p_renderer->setElevation(180);
             p_renderer->setAzimuth(90);
             p_renderer->updateTarget();

        }
    }
}

void UiManager::handleScaleChange(double _scale)
{
//    qDebug() << "UiManager::handleScaleChange " << _scale;

    if (_scale > 0) {
//        s_scale -= 10.0 * _scale/0.1;
        s_scale *= std::pow(10, -_scale/0.1);
    } else if (_scale < 0) {
//        s_scale -= 10.0 * _scale/0.1;
        s_scale /= std::pow(10, _scale/0.1);
    }
//    qDebug() << "UiManager::handleScaleChange new scale:  "  << m_scale;
}

void UiManager::updateCamera(const Vector3d &_tracked)
{
    QVector3D eye = p_renderer->getEye();
    QVector3D ndc = p_renderer->toNDC(QVector3D(_tracked.x, _tracked.y, _tracked.z));
    // QVector3D ndc = p_renderer->toNDC(QVector3D(_tracked.x, _tracked.y -250, _tracked.z));
    ndc.setZ(eye.z());
//    ndc.setZ(0.7);

    p_renderer->setEye(ndc);
    p_renderer->setElevation(180);
    // p_renderer->setElevation(135);
    p_renderer->setAzimuth(90);
    p_renderer->updateTarget(0, 0);
}

void UiManager::setTrackedVehicle(const QString &_id)
{
    m_trackedVehicle = _id;
}

OpenGL::OpenGLCanvas* UiManager::getOpenGLCanvas()
{
    return &m_canvas;
}

OpenGL::OpenGLRenderer* UiManager::getRenderer()
{
    return p_renderer;
}

double UiManager::getScale()
{
    return std::log10(s_scale)/10;
}

/*************************************
 *            PRIVATE SLOTS          *
 ************************************/

void UiManager::onInitialized()
{
    qDebug() << "UiManager::onInitialized";

    p_renderer = p_window->getRenderer();
    m_canvas.setRenderer(p_renderer);

    connect(p_window, SIGNAL(updated()), this, SLOT(onPaint()));

    //
//    drawNodes(&m_canvas);
    drawSegments(&m_canvas);
    drawBuildings(&m_canvas);

    p_renderer->setRenderingStatus(true);
}

void UiManager::onPaint()
{
    //
    for(int i=0; i<m_visualizer.size(); i++)
        m_visualizer.at(i)->update(this);

    delivery::DeliveryListService * dls = delivery::DeliveryListService::getInstance();
    std::vector<std::string> deliveryList = dls->getDeliveryList();
    std::map<std::string, Building*> buildings = World::getInstance()->getBuildings();
    for (std::string buildingId:deliveryList) {
        m_canvas.drawBuilding(buildings.at(buildingId));
    }

    //
    drawVehicles(&m_canvas);

    emit info(p_renderer->info());


}

}
