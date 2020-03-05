#include "standaloneeventscheduler.h"

namespace LIMoSim
{

StandaloneEventScheduler::StandaloneEventScheduler() :
    EventScheduler()
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void StandaloneEventScheduler::scheduleEvent(Event *_event, bool _delay)
{
    double timestamp_s = _event->getTimestamp();
    int index = m_eventQueue.size();
    for(unsigned int i=0; i<m_eventQueue.size(); i++)
    {
        Event *event = m_eventQueue.at(i);
        if(event->getTimestamp()>timestamp_s)
        {
            index = i;
            break;
        }
    }

    m_eventQueue.insert(m_eventQueue.begin()+index, _event);
}

void StandaloneEventScheduler::cancelEvent(Event *_event)
{
    unsigned int i;
    for(i=0; i<m_eventQueue.size(); i++)
    {
        if(_event == m_eventQueue.at(i))
        {
            break;
        }
    }
    if (i < m_eventQueue.size()) {
        m_eventQueue.erase(m_eventQueue.begin()+i);
    }
}

void StandaloneEventScheduler::deleteEvent(Event *_event)
{
    cancelEvent(_event);
    delete _event;
}

/*************************************
 *          SIMULATION CONTROL       *
 ************************************/

void StandaloneEventScheduler::run()
{
    if(m_state!=SIM_STATE::RUNNING)
    {
        m_state = SIM_STATE::RUNNING;
        handleNextEvent();
    }
}

void StandaloneEventScheduler::step()
{
    m_state = SIM_STATE::STOPPED;
    handleNextEvent();
}

void StandaloneEventScheduler::stop()
{
    m_state = SIM_STATE::STOPPED;
    m_eventQueue.clear();
    m_simTime_ms = 0;
}

std::chrono::milliseconds StandaloneEventScheduler::getTime()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch()
                        );
}

void StandaloneEventScheduler::handleNextEvent()
{
    if(m_eventQueue.size()>0)
    {
        Event *event = m_eventQueue.at(0);
        m_eventQueue.pop_front();

        m_eventCount++;
        m_simTime_ms = event->getTimestamp();

        event->handle();
    }

    if (m_state == SIM_STATE::STOPPED) {
        return;
    }


    if(m_eventQueue.size()>0)
    {
        double delta_s = m_eventQueue.at(0)->getTimestamp() - m_simTime_ms;

        if(delta_s==0)
            handleNextEvent();
        else
        {
            //double t = 1;
            //QTimer::singleShot(t, this, SLOT(onUpdated()));
        }

    }
    else
    {
        //std::cout << "StandaloneEventScheduler::handleNextEvent no remaining events" << std::endl;
    }
}


/*************************************
 *            PRIVATE SLOTS          *
 ************************************/

void StandaloneEventScheduler::onUpdated()
{
    handleNextEvent();
}

}
