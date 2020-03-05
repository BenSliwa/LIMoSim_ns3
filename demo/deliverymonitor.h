#ifndef DELIVERYMONITOR_H
#define DELIVERYMONITOR_H

#include <map>
#include <chrono>

#include "LIMoSim/utils/typedefs.h"
#include "LIMoSim/settings/filehandler.h"

namespace LIMoSim {
namespace delivery {

using namespace utils::typedefs;

struct DeliveryStats {
    std::string id;
    std::chrono::milliseconds deliveredAt;
};

class DeliveryMonitor
{
public:
    DeliveryMonitor();

    void registerDelivery(std::string _id);
    void registerDeliveries(StringVector _ids);
    void notifyDelivery(std::string _id);
private:
    std::map<std::string, DeliveryStats> m_deliveryData;
    std::string m_statFileName;
    FileHandler m_statsFile;
};

} // namespace Demo
} // namespace LIMoSim

#endif // DELIVERYMONITOR_H
