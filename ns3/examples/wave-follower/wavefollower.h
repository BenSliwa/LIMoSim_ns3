#ifndef LIMOSIM_NS3_EXAMPLES_WAVEFOLLOWER_WAVEFOLLOWER_H
#define LIMOSIM_NS3_EXAMPLES_WAVEFOLLOWER_WAVEFOLLOWER_H

#include <stdint.h>

namespace LIMoSim {
namespace NS3 {
namespace Examples {
namespace WaveFollower {


void setup(int _runCount = 0,
           uint16_t _simTime_s = 300,
           uint16_t _numOfPairs = 5,
           uint16_t _interPacketInterval_ms = 10,
           uint16_t _packetSize_B = 8000);

} // namespace WaveFollower
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim

#endif // LIMOSIM_NS3_EXAMPLES_WAVEFOLLOWER_WAVEFOLLOWER_H
