#ifndef RAYTRACINGTEST_H
#define RAYTRACINGTEST_H

#include "LIMoSim/simulation/eventhandler.h"
#include "LIMoSim/world/raytracing/raytracing.h"

namespace LIMoSim {
namespace Standalone {

class RaytracingTest : public EventHandler
{
public:
    RaytracingTest();
    ~RaytracingTest();

    void setup(uint8_t _buildingsHeightStart,
               uint8_t _buildingsHeightStep,
               uint8_t _buildingsHeightEnd,
               uint8_t _simTime);
    void step();

    // EventHandler interface
public:
    virtual void initialize() override;
    virtual void handleEvent(Event *_event) override;

private:
    uint8_t m_buildingsHeightStart;
    uint8_t m_buildingsHeightStep;
    uint8_t m_buildingsHeightEnd;
    uint8_t m_buildingsHeightCurrent;
    uint8_t m_simTime;
    Event* m_heartbeat;
    double m_progress;
    std::ofstream * m_raytracingFile;
    Raytracing m_raytracing;

    void logRaytracing(double _timestamp);
};

} // namespace Standalone
} // namespace LIMoSim

#endif // RAYTRACINGTEST_H
