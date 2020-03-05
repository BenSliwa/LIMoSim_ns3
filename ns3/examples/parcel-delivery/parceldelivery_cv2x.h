#ifndef PARCELDELIVERY_CV2X_H
#define PARCELDELIVERY_CV2X_H

#ifdef NS_V2X_BUILD
#include <stdint.h>

namespace LIMoSim {
namespace NS3 {
namespace Examples {
namespace ParcelDelivery {
namespace CV2X {

void setup(uint16_t _runCount = 0,
           uint16_t _numOfUAVs = 1,
           uint16_t _numOfDeliveries = 10);

} // namespace CV2X
} // namespace ParcelDelivery
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim
#endif

#endif // PARCELDELIVERY_CV2X_H
