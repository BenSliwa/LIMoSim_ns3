#ifndef RAYTRACINGPREDICTIONTEST_H
#define RAYTRACINGPREDICTIONTEST_H

#include "LIMoSim/simulation/eventhandler.h"

namespace LIMoSim {
namespace Standalone {

class RaytracingPredictionTest: public EventHandler
{
public:
    RaytracingPredictionTest();

    void setup();
    void step();

    // EventHandler interface
public:
    virtual void initialize() override;
    virtual void handleEvent(Event *_event) override;

private:
    uint8_t m_currentRun;
    Event* m_heartbeat;
    double m_progress;
};

} // namespace Standalone
} // namespace LIMoSim

#endif // RAYTRACINGPREDICTIONTEST_H
