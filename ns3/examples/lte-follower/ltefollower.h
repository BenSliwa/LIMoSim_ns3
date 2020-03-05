#ifndef LTEFOLLOWER_H
#define LTEFOLLOWER_H

#include <stdint.h>

namespace LIMoSim {
namespace NS3 {
namespace Examples {
namespace LteFollower {

void setup(int runCount = 0,
           uint16_t numOfPairs = 1,
           uint16_t simTime_s = 300,
           uint16_t interPacketInterval_ms = 100,
           uint16_t packetSize_B = 190);

} // namespace LteFollower
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim

#endif // LTEFOLLOWER_H
