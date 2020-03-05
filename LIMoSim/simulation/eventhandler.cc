#include "eventhandler.h"
#include "eventscheduler.h"

namespace LIMoSim
{

EventHandler::EventHandler() :
    p_simulation(Simulation::getInstance())
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void EventHandler::scheduleEvent(Event *_event, double _timeIncrement_s)
{
    _event->setTimestamp(_event->getTimestamp() + _timeIncrement_s);
    p_simulation->getEventScheduler()->scheduleEvent(_event);
}

void EventHandler::scheduleEventDelayed(Event *_event)
{
    p_simulation->getEventScheduler()->scheduleEvent(_event, true);
}

void EventHandler::deleteEvent(Event *_event)
{
    p_simulation->getEventScheduler()->deleteEvent(_event);
}


}
