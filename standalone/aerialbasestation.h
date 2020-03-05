#ifndef AERIALBASESTATION_H
#define AERIALBASESTATION_H

#include "LIMoSim/simulation/eventhandler.h"

namespace LIMoSim {
namespace Standalone {

class AerialBaseStation: public EventHandler
{
public:
    AerialBaseStation();

    void setup(uint8_t _runs = 5, uint8_t numCarStart = 1, uint8_t numCarEnd = 5);
    void step();

    // EventHandler interface
public:
    virtual void initialize() override;
    virtual void handleEvent(Event *_event) override;
private:
    uint8_t m_numCarsStart;
    uint8_t m_numCarEnd;
    uint8_t m_runs;
    uint8_t m_currentRun;
    uint8_t m_currentNumCars;
    Event* m_heartbeat;
    double m_progress;
};

} // namespace Standalone
} // namespace LIMoSim

#endif // AERIALBASESTATION_H
