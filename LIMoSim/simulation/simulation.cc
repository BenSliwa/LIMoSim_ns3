#include "simulation.h"

namespace LIMoSim
{

Simulation::Simulation(EventScheduler *_scheduler) :
    p_scheduler(_scheduler)
{

}

Simulation::~Simulation()
{

}

Simulation* Simulation::getInstance(EventScheduler *_scheduler)
{
    static Simulation instance(_scheduler);
    return &instance;
}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void Simulation::run()
{
    p_scheduler->run();
}

void Simulation::step()
{
    p_scheduler->step();
}

void Simulation::stop()
{
    p_scheduler->stop();
}

std::chrono::milliseconds Simulation::getTime()
{
    p_scheduler->getTime();
}

bool Simulation::running()
{
    return p_scheduler->running();
}

EventScheduler* Simulation::getEventScheduler()
{
    return p_scheduler;
}

std::string Simulation::getName()
{
    return m_name;
}

Simulation* Simulation::setName(std::string _name)
{
    m_name = _name;
    return this;
}

int Simulation::getRunCount()
{
    return m_runCount;
}

Simulation* Simulation::setRunCount(int _runCount)
{
    m_runCount = _runCount;
    return this;
}

}
