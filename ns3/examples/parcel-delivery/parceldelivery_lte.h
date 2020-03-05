#ifndef PARCELDELIVERY_LTE_H
#define PARCELDELIVERY_LTE_H


#include <stdint.h>

namespace LIMoSim {
namespace NS3 {
namespace Examples {
namespace ParcelDelivery {
namespace LTE {

void setup(uint16_t _runCount = 0,
           uint16_t _numOfUAVs = 1,
           uint16_t _numOfDeliveries = 10,
           uint16_t _numOfDisturbers = 20,
           uint16_t _disturberInterPacketInterval_ms = 100,
           uint16_t _disturberPacketSize_B = 200);

} // namespace LTE
} // namespace ParcelDelivery
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim

#endif // PARCELDELIVERY_LTE_H
