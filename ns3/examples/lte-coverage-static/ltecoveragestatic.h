#ifndef LTECOVERAGESTATIC_H
#define LTECOVERAGESTATIC_H

#include <stdint.h>

namespace LIMoSim {
namespace NS3 {
namespace Examples {
namespace LteCoverageStatic {

void setup(int _runCount = 0,
           uint16_t _simTime_s = 300,
           uint16_t _numOfUes = 5,
           uint16_t _interPacketInterval_ms = 10,
           uint16_t _packetSize_B = 8000);

} // namespace LteCoverageStatic
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim

#endif // LTECOVERAGESTATIC_H
