#ifndef LTESIMPLEEPCMOBILE_H
#define LTESIMPLEEPCMOBILE_H


#include <ns3/application.h>

#include "LIMoSim/mobility/uav/reynolds/behavior_seek.h"

namespace LIMoSim {
namespace NS3 {
namespace Examples {

using namespace ns3;

namespace LteSimpleEpcMobile {

void setup(int runCount = 0);

void callbackLteReportUeMeasurement (std::string path,uint16_t rnti, uint16_t cellId,
                                     double rsrp, double rsrq, bool servingCell, uint8_t componentCarrierId);

// Custom Behavior
class SimpleBehavior: public Behavior {

public:
    SimpleBehavior(std::string _agentId = "");

    // Behavior interface
public:
    Steering apply();
    virtual void setAgent(std::string _agentId);

private:
    std::string m_agentId;
    bool m_trip;
    Behavior_Seek m_behavior_trip;
    Behavior_Seek m_behavior_return;

};
}

}
}
}
#endif // LTESIMPLEEPCMOBILE_H
