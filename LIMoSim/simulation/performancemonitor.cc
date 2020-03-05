#include "performancemonitor.h"
#include <fstream>

namespace LIMoSim {

PerformanceMonitor *PerformanceMonitor::getInstance()
{
    static PerformanceMonitor instance;
    return &instance;
}

PerformanceMonitor::~PerformanceMonitor()
{
    if (m_performanceCheck) {
        deleteEvent(m_performanceCheck);
    }

    if (m_exportEnabled) {
        m_exportFileStream->close();
        delete m_exportFileStream;
    }
}

void PerformanceMonitor::setup()
{
    if (!m_initialized && !Simulation::getInstance()->running()) {
        std::cout << "initializing performance monitor" << std::endl;
        initialize();
        m_initialized = true;
    } else {
        std::cerr << "Performance monitoring must be enabled before simulation start" << std::endl;
    }
}

void PerformanceMonitor::setupWithExport(std::string _filename)
{
    if (!m_initialized && !Simulation::getInstance()->running()) {
        std::cout << "initializing performance monitor with export" << std::endl;
        m_exportEnabled = true;
        m_exportFilename = _filename;
        m_exportFileStream = new std::ofstream();
        m_exportFileStream->open(m_exportFilename, std::ios::trunc);
        if (m_exportFileStream->is_open()) {
            (*m_exportFileStream) << "time\t" << "elapsed" << std::endl;
        } else {
            std::cout << "error in performance monitor - could not open export file: " << m_exportFilename << std::endl;
        }

        initialize();
        m_initialized = true;
    } else {
        std::cerr << "Performance monitoring must be enabled only once and before simulation start" << std::endl;
    }
}

void PerformanceMonitor::reset()
{
    if (m_performanceCheck) {
        deleteEvent(m_performanceCheck);
    }

    if (m_exportEnabled) {
        m_exportFileStream->close();
        delete m_exportFileStream;
        m_exportEnabled = false;
        m_exportFilename = "";
    }

    m_initialized = false;
}

void PerformanceMonitor::initialize()
{
    m_checkCount = 0;
    m_performanceCheck = new Event(1, this, "PerformanceCheck");
    scheduleEvent(m_performanceCheck);
}

void PerformanceMonitor::handleEvent(Event *_event)
{
    if (_event->getInfo() == m_performanceCheck->getInfo() && Simulation::getInstance()->running()) {
        scheduleEvent(m_performanceCheck, 1);
        checkPerformance();
    }
}

PerformanceMonitor::PerformanceMonitor():
    EventHandler (),
    m_performanceCheckInterval_s(1),
    m_initialized(false)
{

}

void PerformanceMonitor::checkPerformance()
{
    if (m_checkCount++) {
        auto lastCheck = m_lastCheck;
        m_lastCheck = std::chrono::system_clock::now();
        std::chrono::duration<double> duration = std::chrono::system_clock::now() - lastCheck;
//        std::cout <<"elapsed seconds since last simulated second: " << duration.count() << std::endl;

        if (m_exportEnabled) {
            exportPerformance(duration);
        }
    } else {
        m_lastCheck = std::chrono::system_clock::now();
    }
}

void PerformanceMonitor::exportPerformance(std::chrono::duration<double> duration)
{
    (*m_exportFileStream) << m_checkCount -1 << "\t" << duration.count() << std::endl;
}

} // namespace LIMoSim
