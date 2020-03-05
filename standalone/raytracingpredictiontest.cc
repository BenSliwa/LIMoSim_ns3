#include "raytracingpredictiontest.h"

#include "scenarios.h"

#include "LIMoSim/simulation/simulation.h"
#include "LIMoSim/world/vehiclemanager.h"
#include "LIMoSim/world/raytracing/raytracingprediction.h"

namespace LIMoSim {
namespace Standalone {
using namespace scenarios;

RaytracingPredictionTest::RaytracingPredictionTest():
    EventHandler ()
{

}

void RaytracingPredictionTest::setup()
{
    m_currentRun = 0;
    step();
}

void RaytracingPredictionTest::step()
{
    using namespace raytracing;
    Simulation::getInstance()->stop();

    double h = 10;

    Simulation::getInstance()->setName("RandomWaypoint")
            ->setRunCount(m_currentRun);
    std::vector<double> predictionHorizons {5,10,15,20,25,30};
    std::vector<int> heights {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50};
    RaytracingPrediction::getInstance(heights, predictionHorizons);

    VehicleManager::getInstance()->clearVehicles();
    randomWaypointScenario_1(h);


    Simulation::getInstance()->getEventScheduler()->scheduleEvent(
                new Event(600, this, "Stop"));
    m_heartbeat = new Event(1, this, "progress");
    scheduleEvent(m_heartbeat,1);
    m_progress = 0;
    Simulation::getInstance()->run();

}

void RaytracingPredictionTest::initialize()
{

}

void RaytracingPredictionTest::handleEvent(Event *_event)
{
    if (_event->getInfo() == "Stop") {
        std::cout << "handling scenario event " << _event->getInfo() << std::endl;
        deleteEvent(m_heartbeat);
        raytracing::RaytracingPrediction::getInstance()->reset();
        VehicleManager::getInstance()->clearVehicles();

//        step();
    }
    if (_event->getInfo() == "progress") {
        std::cout << Simulation::getInstance()->getName()
                  << "_r" << Simulation::getInstance()->getRunCount()
                  << " progress: " << ++m_progress << "seconds" << std::endl;
        scheduleEvent(m_heartbeat,1);
    }
}

} // namespace Standalone
} // namespace LIMoSim
