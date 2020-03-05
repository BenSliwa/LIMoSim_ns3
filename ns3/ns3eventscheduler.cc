#include "ns3eventscheduler.h"

namespace LIMoSim {
namespace NS3 {

NS3EventScheduler::NS3EventScheduler():
    EventScheduler()
{

}


void NS3EventScheduler::scheduleEvent(Event *_event, bool _delay)
{
    Time delay;
    if (_delay) {
        delay = Time::FromDouble ((_event->getTimestamp()), Time::S);
    } else {
        delay = Time::FromDouble ((_event->getTimestamp()), Time::S) - Now();
    }
    EventId eventId = Simulator::Schedule(
                delay,
                &NS3EventScheduler::handleEvent,
                this,
                _event
                );
    m_scheduledLIMoSimEvents[_event] = eventId;
}

void NS3EventScheduler::cancelEvent(Event *_event)
{
    auto it = m_scheduledLIMoSimEvents.find(_event);
    if (it != m_scheduledLIMoSimEvents.end()) {
        Simulator::Cancel((*it).second);
        m_scheduledLIMoSimEvents.erase(it);
    }
}

void NS3EventScheduler::deleteEvent(Event *_event)
{
    cancelEvent(_event);
    delete _event;
}

void NS3EventScheduler::handleEvent(Event *_event)
{
    auto it = m_scheduledLIMoSimEvents.find(_event);
    if (it != m_scheduledLIMoSimEvents.end()) {
        m_scheduledLIMoSimEvents.erase(it);
    }
    _event->handle();
}



void NS3EventScheduler::run()
{
    if (m_state != SIM_STATE::RUNNING) {
        m_state = SIM_STATE::RUNNING;
    }
    std::cout << "starting ns3 simulation ..." <<std::endl;
    Simulator::Run ();

    m_state = SIM_STATE::STOPPED;
    Simulator::Destroy();
    std::cout << "ns3 simulation finished." <<std::endl;
}

void NS3EventScheduler::step()
{
}

void NS3EventScheduler::stop()
{
    m_state = SIM_STATE::STOPPED;
    Simulator::Stop();
    std::cout << "ns3 simulation stopped." <<std::endl;
}

std::chrono::milliseconds NS3EventScheduler::getTime()
{
//    return  std::chrono::duration_cast<std::chrono::milliseconds>(
//                std::chrono::duration<long>(ns3::Simulator::Now().GetMilliSeconds())
//                );
    return  std::chrono::duration<long, std::ratio<1,1000>>(ns3::Simulator::Now().GetMilliSeconds());
}

} // namespace NS3
} // namespace LIMoSIM
