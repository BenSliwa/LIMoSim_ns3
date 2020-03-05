#include "aerialbasestation.h"

#include "scenarios.h"

#include "LIMoSim/simulation/simulation.h"
#include "LIMoSim/world/vehiclemanager.h"

namespace LIMoSim {
namespace Standalone {
using namespace scenarios;

AerialBaseStation::AerialBaseStation():
    EventHandler (),
    m_numCarsStart(1),
    m_numCarEnd(5),
    m_runs(5)
{
}

void AerialBaseStation::setup(uint8_t _runs, uint8_t numCarStart, uint8_t numCarEnd)
{
    m_numCarsStart = numCarStart;
    m_numCarEnd = numCarEnd;
    m_runs = _runs;
    m_currentRun = 0;
    m_currentNumCars = m_numCarsStart;
    step();
}

void AerialBaseStation::step()
{
    Simulation::getInstance()->stop();

   if(m_currentNumCars <= m_numCarEnd){
    if (m_currentRun < m_runs) {
        VehicleManager::getInstance()->clearVehicles();
        aerialBaseStation(m_currentNumCars, m_currentRun++);

        if (m_currentRun >= m_runs && m_currentNumCars <= m_numCarEnd) {
            m_currentRun = 0;
            m_currentNumCars++;
        }

        Simulation::getInstance()->getEventScheduler()->scheduleEvent(
                    new Event(300, this, "Stop"));
        m_heartbeat = new Event(1, this, "progress");
        scheduleEvent(m_heartbeat,1);
        m_progress = 0;
        Simulation::getInstance()->run();
    }
   }
}



void AerialBaseStation::initialize()
{
}

void AerialBaseStation::handleEvent(LIMoSim::Event *_event)
{
    if (_event->getInfo() == "Stop") {
        std::cout << "handling scenario event " << _event->getInfo() << std::endl;
        deleteEvent(m_heartbeat);
        step();
    }
    if (_event->getInfo() == "progress") {
        std::cout << "progress: " << ++m_progress << "seconds" << std::endl;scheduleEvent(m_heartbeat,1);
    }
}

} // namespace Standalone
} // namespace LIMoSim
