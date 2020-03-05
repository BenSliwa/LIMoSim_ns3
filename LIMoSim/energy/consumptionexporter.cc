#include "consumptionexporter.h"
#include "LIMoSim/simulation/simulation.h"

namespace LIMoSim {
namespace Energy {

ConsumptionExporter *ConsumptionExporter::getInstance()
{
    static ConsumptionExporter _instance;
    return &_instance;
}

ConsumptionExporter::~ConsumptionExporter()
{
    for (FileStreamMap::iterator entry = m_vehicleFiles.begin(); entry != m_vehicleFiles.end(); entry++) {
        entry->second->close();
        delete entry->second;
        m_vehicleFiles.erase(entry);
    }
}

void ConsumptionExporter::registerVehicle(std::string _vehicleId)
{
    if (m_vehicleFiles.count(_vehicleId)) {
        return;
    }
    std::string filename = "../results/Energy_" +
            Simulation::getInstance()->getName() +
            "_Vh" + _vehicleId + "_" +
            std::to_string(Simulation::getInstance()->getRunCount()) + ".csv";

    m_vehicleFiles[_vehicleId] = new std::ofstream();
    m_vehicleFiles.at(_vehicleId)->open(filename, std::ios::trunc);
    if (m_vehicleFiles.at(_vehicleId)->is_open()) {
        (*m_vehicleFiles.at(_vehicleId)) << "time\t" << "energy" << std::endl;
    } else {
        std::cout << "error in energy consumption export - could not open export file: " << filename << std::endl;
    }
}

void ConsumptionExporter::unregisterVehicle(std::string _vehicleId)
{
    if (m_vehicleFiles.count(_vehicleId)) {
        m_vehicleFiles.at(_vehicleId)->close();
        delete m_vehicleFiles.at(_vehicleId);
        m_vehicleFiles.erase(_vehicleId);
    }
}

void ConsumptionExporter::exportVehicleConsumption(std::string _vehicleId, Consumption _consumption)
{
    if (m_vehicleFiles.count(_vehicleId)) {
        (*m_vehicleFiles.at(_vehicleId)) << _consumption.getTimestamp() << "\t" << _consumption.getEnergy() << std::endl;
    }
}

ConsumptionExporter::ConsumptionExporter()
{

}

} // namespace Energy
} // namespace LIMoSim
