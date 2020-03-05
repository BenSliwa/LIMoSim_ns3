#ifndef LTESIMPLEEPCSTATIC_H
#define LTESIMPLEEPCSTATIC_H

#include <string>

namespace LIMoSim {
namespace NS3 {
namespace Examples {

namespace LteSimpleEpcStatic {

void setup();

void NotifyRandomAccessSuccessfulUe(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti);
void ConnectSrb0Traces(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti);
void NotifyConnectionEstablishedUe(std::string context,
                                   uint64_t imsi,
                                   uint16_t cellid,
                                   uint16_t rnti);

void NotifyConnectionEstablishedEnb (std::string context,
                                uint64_t imsi,
                                uint16_t cellid,
                                uint16_t rnti);

void Lte_RlcDlRxPDU (std::string context, uint16_t rnti, uint8_t lcid, uint32_t packetSize, uint64_t delay);
}

}
}
}
#endif // LTESIMPLEEPCSTATIC_H
