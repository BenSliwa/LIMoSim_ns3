#ifndef NS3EVENTSCHEDULER_H
#define NS3EVENTSCHEDULER_H

#include "LIMoSim/simulation/eventscheduler.h"
#include <map>
#include "ns3/core-module.h"

namespace LIMoSim {

namespace NS3 {

using namespace ns3;

class NS3EventScheduler : public EventScheduler
{
public:
    NS3EventScheduler();

    // EventScheduler interface
public:
    virtual void scheduleEvent(Event *_event, bool _delay = false);
    virtual void cancelEvent(Event *_event);
    virtual void deleteEvent(Event *_event);

    // EventScheduler interface
public:
    virtual void run();
    virtual void step();
    virtual void stop();

    std::chrono::milliseconds getTime();

protected:
    void handleEvent(Event *_event);

private:
    std::map<Event*, EventId> m_scheduledLIMoSimEvents;
};

} // namespace NS3
} // namespace LIMoSIM

#endif // NS3EVENTSCHEDULER_H
