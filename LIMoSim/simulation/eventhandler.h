#ifndef LIMOSIM_EVENTHANDLER_H
#define LIMOSIM_EVENTHANDLER_H

#include "event.h"
#include "simulation.h"

namespace LIMoSim
{

class EventHandler
{
public:
    EventHandler();

    //
    virtual void initialize() = 0;
    virtual void handleEvent(Event *_event) = 0;

    //
    void scheduleEvent(Event *_event, double _timeIncrement_s = 0);
    void scheduleEventDelayed(Event *_event);
    void deleteEvent(Event *_event);

private:
    Simulation *p_simulation;
};

}

#endif // LIMOSIM_EVENTHANDLER_H
