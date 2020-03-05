#ifndef ROADUTILS_H
#define ROADUTILS_H

#include "LIMoSim/mobility/car/car.h"

namespace LIMoSim {
namespace RoadUtils {

void init();
void resetLane();

Gate* randomValidGate();

RoadPosition randomValidRoadPosition();

RoadPosition sameRoadPosition();

} // namespace RoadUtils
} // namespace LIMoSim

#endif // ROADUTILS_H
