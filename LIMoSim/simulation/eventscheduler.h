#ifndef LIMOSIM_EVENTSCHEDULER_H
#define LIMOSIM_EVENTSCHEDULER_H

#include "event.h"
#include <iostream>
#include <chrono>

namespace LIMoSim
{

namespace SIM_STATE
{
    enum{
        RUNNING,
        STOPPED
    };
}

class EventScheduler
{
public:
    EventScheduler();

    // event handling
    virtual void scheduleEvent(Event *_event, bool _delay =false) = 0;
    virtual void cancelEvent(Event *_event) = 0;
    virtual void deleteEvent(Event *_event) = 0;

    virtual std::chrono::milliseconds getTime() = 0;

    // simulation control
    virtual void run();
    virtual void step();
    virtual void stop();



    bool running();

protected:
    int m_state;
    int m_eventCount;
};

}

#endif // LIMOSIM_EVENTSCHEDULER_H
