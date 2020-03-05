#ifndef PARCELDELIVERY_WAVE_H
#define PARCELDELIVERY_WAVE_H


#include <stdint.h>

namespace LIMoSim {
namespace NS3 {
namespace Examples {
namespace ParcelDelivery {
namespace Wave {

void setup(uint16_t _runCount = 0,
           uint16_t _numOfUAVs = 1,
           uint16_t _numOfDeliveries = 10);
} // namespace Wave
} // namespace ParcelDelivery
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim

#endif // PARCELDELIVERY_WAVE_H
