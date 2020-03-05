#ifndef LTECOVERAGEMOBILE_H
#define LTECOVERAGEMOBILE_H

#include <stdint.h>

namespace LIMoSim {
namespace NS3 {
namespace Examples {
namespace LteCoverageMobile {

void setup(int runCount = 0,
           uint16_t simTime_s = 300,
           uint16_t numOfUes = 5,
           uint16_t interPacketInterval_ms = 10,
           uint16_t packetSize_B = 8000);

} // namespace LteCoverageMobile
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim

#endif // LTECOVERAGEMOBILE_H
