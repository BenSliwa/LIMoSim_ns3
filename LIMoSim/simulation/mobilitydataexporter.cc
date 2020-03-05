#include "mobilitydataexporter.h"

#include <iomanip>
#include <ctime>

#include "LIMoSim/world/vehiclemanager.h"


namespace LIMoSim {

MobilityDataExporter::MobilityDataExporter(Vehicle *_vehicle, double _captureInterval_s):
    m_captureInterval_s(_captureInterval_s),
    m_running(false),
    m_vehicle(_vehicle)
{
    m_outputFilename = "../results/Mobility_" +
            Simulation::getInstance()->getName() +
            "_Vh" + _vehicle->getId() + "_" +
            std::to_string(Simulation::getInstance()->getRunCount()) + ".txt";

    m_outputFile = new std::ofstream();
    m_outputFile->open(m_outputFilename.c_str(), std::ios::trunc);
    if (!m_outputFile->is_open()) {
        std::cout << "error in mobility data exporter - could not open export file: " << m_outputFilename << std::endl;
    } else {
        (*m_outputFile) << "time\t"
                        << "pos.x\t" << "pos.y\t" << "pos.z\t"
                        << "vel.x\t" << "vel.y\t" << "vel.z\t"
                        << "acc.x\tacc.y\tacc.z\t"
                        << "ori.r\tori.p\tori.y\t"
                        << "avl.r\tavl.p\tavl.y\t"
                        << std::endl;
    }
}

MobilityDataExporter::~MobilityDataExporter()
{
    m_outputFile->close();
    delete m_outputFile;
    if (m_running) {
        deleteEvent(m_captureTimer);
    }
}

void MobilityDataExporter::start()
{
    initialize();
}

void MobilityDataExporter::initialize()
{
    m_captureTimer = new Event(0.25, this, "MobilityExport");
    scheduleEvent(m_captureTimer);
    m_running = true;
}

void MobilityDataExporter::handleEvent(Event *_event)
{
    if (_event->getInfo() == m_captureTimer->getInfo()) {
        exportMobilityData(_event->getTimestamp(), captureMobilityData());
        scheduleEvent(m_captureTimer, m_captureInterval_s);
    }
}

MobilityData MobilityDataExporter::captureMobilityData()
{
    return MobilityData(
                m_vehicle->getPosition(),
                m_vehicle->getOrientation(),
                m_vehicle->getVelocity(),
                m_vehicle->getOrientationVelocity(),
                m_vehicle->getAcceleration(),
                m_vehicle->getType() + m_vehicle->getId()
                );
}

void MobilityDataExporter::exportMobilityData(double _timestamp, MobilityData _mobilityData)
{
    (*m_outputFile) << _timestamp << "\t"
                    << _mobilityData.position.toString("\t") << "\t"
                    << _mobilityData.velocity.toString("\t") << "\t"
                    << _mobilityData.acceleration.toString("\t") << "\t"
                    << _mobilityData.orientation.toString("\t") << "\t"
                    << _mobilityData.orientationVelocity.toString("\t") << "\t"
                    << std::endl;
}

} // namespace LIMoSim
