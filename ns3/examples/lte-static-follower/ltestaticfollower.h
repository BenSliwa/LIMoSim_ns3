#ifndef LIMOSIM_NS3_EXAMPLES_LTESTATICFOLLOWER_LTESTATICFOLLOWER_H
#define LIMOSIM_NS3_EXAMPLES_LTESTATICFOLLOWER_LTESTATICFOLLOWER_H

#include <stdint.h>

namespace LIMoSim {
namespace NS3 {
namespace Examples {
namespace LteStaticFollower {

void setup(int _runCount = 0,
           uint16_t _simTime_s = 300,
           uint16_t _numOfUes = 5,
           uint16_t _interPacketInterval_ms = 10,
           uint16_t _packetSize_B = 8000);

} // namespace LteStaticFollower
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim

#endif // LIMOSIM_NS3_EXAMPLES_LTESTATICFOLLOWER_LTESTATICFOLLOWER_H
