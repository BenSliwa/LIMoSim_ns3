#ifndef PERFORMANCEMONITOR_H
#define PERFORMANCEMONITOR_H


#include "LIMoSim/simulation/eventhandler.h"

#include <chrono>

namespace LIMoSim {

class PerformanceMonitor: public EventHandler
{
public:
    static PerformanceMonitor* getInstance();
    virtual ~PerformanceMonitor();

    // Monitoring control
    void setup();
    void setupWithExport(std::string _filename = "./performance.csv");
    void reset();

protected:
    void initialize();
    void handleEvent(Event *_event);

private:
    PerformanceMonitor();

    /**
     * @brief m_lastCheck
     * The last point in time a whole second was simulated.
     */
    std::chrono::system_clock::time_point m_lastCheck;

    /**
     * @brief m_secondTimer
     * Event to pick up every simulated second
     * and trigger the performance measurement.
     */
    Event *m_performanceCheck;

    double m_performanceCheckInterval_s;

    uint64_t m_checkCount;

    bool m_initialized;

    bool m_exportEnabled;
    std::string m_exportFilename;
    std::ofstream *m_exportFileStream;

    void checkPerformance();
    void exportPerformance(std::chrono::duration<double> duration);
};

} // namespace LIMoSim

#endif // PERFORMANCEMONITOR_H
