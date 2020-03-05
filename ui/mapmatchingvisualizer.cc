#include "mapmatchingvisualizer.h"
#include "LIMoSim/world/vehiclemanager.h"
#include "LIMoSim/mobility/car/strategic/strategicmodel.h"
#include "uimanager.h"

namespace LIMoSim
{

MapMatchingVisualizer::MapMatchingVisualizer() :
    Visualizer()
{
    m_trace.load("m5t_18_30_t2_5.txt");
}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void MapMatchingVisualizer::update(UiManager *_ui)
{
    if(!_ui)
    {
        m_trace.getNext();
        return;
    }

    TraceEntry entry = m_trace.getCurrent();
    //entry.info();
    Vector3d p = entry.position;

    MapMatching matching;
    MatchedEntry matched = matching.match(entry.position, entry.heading);

    _ui->updateCamera(p);


    _ui->getOpenGLCanvas()->buildTriangle(p, 90 - entry.heading, 5, 15);


    VehicleManager *vehicleManager = VehicleManager::getInstance();
    Car *car0 = 0;
    if(vehicleManager->hasVehicle("C0"))
        car0 = dynamic_cast<Car*>(vehicleManager->getVehicle("C0"));
    else
        car0 = vehicleManager->createCar("C0");

    RoadPosition roadPosition;
    roadPosition.laneSegment = matched.lane;
    roadPosition.offset_m = matched.offset_m;
    car0->setRoadPosition(roadPosition);
    car0->getStrategicModel()->initialize(); // call for surrogate vehicles


    // perform mobility prediction
    float horizon_s = 5.0;
    float interval_s = 0.1;
    int steps = horizon_s/interval_s;
    for(int i=0; i<steps; i++)
    {
        car0->move(interval_s);
    }

    OpenGL::OpenGLCanvas *canvas = _ui->getOpenGLCanvas();
    Vector3d m = car0->getPosition();
    OpenGLShape shape("white");
    float z = 1;
    shape.addVertex(canvas->fromVector(m + Vector3d(-1, -1, z)));
    shape.addVertex(canvas->fromVector(m + Vector3d(-1, 1, z)));
    shape.addVertex(canvas->fromVector(m + Vector3d(1, 1, z)));
    shape.addVertex(canvas->fromVector(m + Vector3d(1, -1, z)));
    _ui->getRenderer()->drawCube(shape, 2.5, false);


    car0->setRoadPosition(roadPosition); // reset the road position
}

}
