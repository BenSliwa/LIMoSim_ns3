#ifndef LIMOSIM_SIMULATION_H
#define LIMOSIM_SIMULATION_H

#include "eventscheduler.h"

namespace LIMoSim
{

class Simulation
{
public:
    Simulation(EventScheduler *_scheduler = 0);
    ~Simulation();

    static Simulation* getInstance(EventScheduler *_scheduler = 0);

    // simulation control
    void run();
    void step();
    void stop();

    std::chrono::milliseconds getTime();

    bool running();

    //
    EventScheduler* getEventScheduler();

    std::string getName();
    Simulation* setName(std::string _name);

    int getRunCount();
    Simulation* setRunCount(int _runCount);

private:
    EventScheduler *p_scheduler;
    std::string m_name;
    int m_runCount;
};

}

#endif // LIMOSIM_SIMULATION_H
