#ifndef LIMOSIM_STANDALONEEVENTSCHEDULER_H
#define LIMOSIM_STANDALONEEVENTSCHEDULER_H

#include "LIMoSim/simulation/eventscheduler.h"
#include <deque>

#include <QObject>
#include <QTimer>

namespace LIMoSim
{

class StandaloneEventScheduler : public QObject, public EventScheduler
{
    Q_OBJECT
public:
    StandaloneEventScheduler();

    // inherited
    void scheduleEvent(Event *_event, bool _delay = false);
    void cancelEvent(Event *_event);
    void deleteEvent(Event *_event);

    // simulation control
    void run();
    void step();
    void stop();

    std::chrono::milliseconds getTime();

    //
    void handleNextEvent();

private slots:
    void onUpdated();

private:
    std::deque<Event*> m_eventQueue;
    int m_eventCount;
    double m_simTime_ms;
};

}

#endif // LIMOSIM_STANDALONEEVENTSCHEDULER_H
