#ifndef MOBILITYDATAEXPORTER_H
#define MOBILITYDATAEXPORTER_H

#include <iostream>
#include <fstream>
#include <vector>
#include "LIMoSim/simulation/eventhandler.h"
#include "LIMoSim/mobility/mobilitydata.h"


namespace LIMoSim {

class Vehicle;

class MobilityDataExporter: public EventHandler
{
public:
    MobilityDataExporter(LIMoSim::Vehicle *_vehicle, double _captureInterval_ms);
    ~MobilityDataExporter();

    //
    void start();

protected:
    void initialize();
    void handleEvent(Event *_event);

    MobilityData captureMobilityData();
    void exportMobilityData(double _timestamp, MobilityData _mobilityData);

private:
    double m_captureInterval_s;
    bool m_running;
    std::ofstream* m_outputFile;
    std::string m_outputFilename;
    Vehicle *m_vehicle;

    Event *m_captureTimer;

};

} // namespace LIMoSim

#endif // MOBILITYDATAEXPORTER_H
