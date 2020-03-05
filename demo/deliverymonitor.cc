#include "deliverymonitor.h"

#include <iostream>

#include "LIMoSim/simulation/simulation.h"

namespace LIMoSim {
namespace delivery {

DeliveryMonitor::DeliveryMonitor()
{

    Simulation* sim = Simulation::getInstance();
    m_statFileName = std::string("../results/") +
            "ParcelDeliveryStats_" +
            sim->getName() + "_" +
            std::to_string(sim->getRunCount()) +
            ".txt";
    m_statsFile.write(
                "#ID\tDeliveryTime\n",
                m_statFileName);
}

void DeliveryMonitor::registerDelivery(std::string _id)
{
    m_deliveryData[_id] = DeliveryStats();
}

void DeliveryMonitor::registerDeliveries(StringVector _ids)
{
    for (auto id : _ids) {
        m_deliveryData[id] = DeliveryStats();
    }
}

void DeliveryMonitor::notifyDelivery(std::string _id)
{
    if (m_deliveryData.count(_id)) {        
        std::cout << "DeliveryMonitor::notifyDelivery registering delivery completion for " << _id << std::endl;
//        m_deliveryData.at(_id).deliveredAt = std::chrono::duration_cast<std::chrono::milliseconds>(
//                    std::chrono::system_clock::now().time_since_epoch()
//                    );
        m_deliveryData.at(_id).deliveredAt = Simulation::getInstance()->getTime();
        FileHandler::append(
                    _id + "\t" +
                    std::to_string(m_deliveryData.at(_id).deliveredAt.count()) + "\n",
                    m_statFileName);
    }
}


} // namespace Demo
} // namespace LIMoSim
