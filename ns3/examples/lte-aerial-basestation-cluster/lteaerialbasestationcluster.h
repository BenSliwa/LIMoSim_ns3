#ifndef LTEAERIALBASESTATIONCLUSTER_H
#define LTEAERIALBASESTATIONCLUSTER_H

#include <stdint.h>

namespace LIMoSim {
namespace NS3 {
namespace Examples {
namespace LteAerialBasestationCluster {

void setup(int runCount = 0,
           uint16_t numOfUAVs = 2,
           uint16_t numOfCars = 6,
           uint16_t simTime_s = 300,
           uint16_t interPacketInterval_ms = 500,
           uint16_t packetSize_B = 20);

} // namespace LteAerialBasestationCluster
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim

#endif // LTEAERIALBASESTATIONCLUSTER_H
