#include "eventscheduler.h"

namespace LIMoSim
{

EventScheduler::EventScheduler() :
    m_state(SIM_STATE::STOPPED),
    m_eventCount(0)
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void EventScheduler::run()
{

}

void EventScheduler::step()
{

}

void EventScheduler::stop()
{

}

bool EventScheduler::running()
{
    return m_state == SIM_STATE::RUNNING;
}

}
