#ifndef PARCELDELIVERY_H
#define PARCELDELIVERY_H

#include <stdint.h>

namespace LIMoSim {
namespace NS3 {
namespace Examples {
namespace ParcelDelivery {

void setup(uint16_t _runCount = 0,
           uint16_t _numOfUAVs = 1,
           uint16_t _numOfDeliveries = 10);

} // namespace ParcelDelivery
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim

#endif // PARCELDELIVERY_H
