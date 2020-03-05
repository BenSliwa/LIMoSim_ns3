#include "raytracingtest.h"

#include "scenarios.h"

#include "LIMoSim/mobility/uav/reynolds/behaviors.h"
#include "LIMoSim/simulation/simulation.h"
#include "LIMoSim/world/vehiclemanager.h"
#include "LIMoSim/world/road/roadutils.h"
#include "ui/data/uidatamanager.h"

namespace LIMoSim {
namespace Standalone {

using namespace UI::Data;

RaytracingTest::RaytracingTest()
{

}

RaytracingTest::~RaytracingTest()
{

}

void RaytracingTest::setup(uint8_t _buildingsHeightStart,
                           uint8_t _buildingsHeightStep,
                           uint8_t _buildingsHeightEnd,
                           uint8_t _simTime)
{
    m_buildingsHeightStart = _buildingsHeightStart;
    m_buildingsHeightStep = _buildingsHeightStep;
    m_buildingsHeightEnd = _buildingsHeightEnd;
    m_buildingsHeightCurrent = 0;
    m_simTime = _simTime;
    m_raytracingFile = nullptr;
    step();
}

void RaytracingTest::step()
{
    Simulation::getInstance()->stop();

    if (m_raytracingFile && m_raytracingFile->is_open()){
        m_raytracingFile->close();
        delete m_raytracingFile;
        m_raytracingFile = nullptr;
    }

    if(m_buildingsHeightCurrent <= m_buildingsHeightEnd) {
        VehicleManager::getInstance()->clearVehicles();

        m_raytracingFile = new std::ofstream();
        m_raytracingFile->open("../results/raytracing_" +
                               std::to_string(m_buildingsHeightCurrent) +
                               ".txt");
        //m_raytracing.clearCache();

        // Scenario start
        VehicleManager *vehicleManager = VehicleManager::getInstance();
        UIDataManager *uidm = UIDataManager::getInstance();

        vehicleManager->setDeterministicCarPathEnabled(true);
        vehicleManager->enableMobilityBroadcastHelper();

        RoadPosition roadPosition;

        Car *car = vehicleManager->createCar("C0");
        uidm->getVehicleData(car->getId())->getShape()->setColor("red");
        car->initialize();


        UAV *uav = vehicleManager->createUAV("U0");
        uidm->getVehicleData(uav->getId())->getShape()->setColor("yellow");
        uav->setPosition(Vector3d(0, 0, 40));
        uav->setBehavior(new Behavior_NoOp());
        // Scenario end


        Simulation::getInstance()->getEventScheduler()->scheduleEvent(
                    new Event(m_simTime, this, "Stop"));
        m_heartbeat = new Event(1, this, "progress");
        scheduleEvent(m_heartbeat,0);
        m_progress = 0;
        Simulation::getInstance()->run();
    }
}

void RaytracingTest::initialize()
{

}

void RaytracingTest::handleEvent(Event *_event)
{
    if (_event->getInfo() == "Stop") {
        std::cout << "handling scenario event " << _event->getInfo() << std::endl;
        deleteEvent(m_heartbeat);
        m_buildingsHeightCurrent += m_buildingsHeightStep;
        step();
    }
    if (_event->getInfo() == "progress") {
        std::cout << "progress: " << ++m_progress << "seconds" << std::endl;
        logRaytracing(_event->getTimestamp());
        scheduleEvent(m_heartbeat,1);
    }
}

void RaytracingTest::logRaytracing(double _timestamp)
{
    /*
    VehicleManager::getInstance()->getVehicle("U0")->getPosition();
    RayTrace trace = m_raytracing.trace(VehicleManager::getInstance()->getVehicle("U0")->getPosition(),
                                        VehicleManager::getInstance()->getVehicle("C0")->getPosition(),
                                        m_buildingsHeightCurrent);

    (*m_raytracingFile) << _timestamp << ","
                        << trace.distance_m << ","
                        << trace.attenuated_m << ","
                        << trace.intersections.size() << "\n";*/
}

} // namespace Standalone
} // namespace LIMoSim
