#ifndef ROUTINGUTILS_H
#define ROUTINGUTILS_H

#include "LIMoSim/mobility/routing/types.h"


namespace LIMoSim {
namespace mobility {
namespace routing {

using namespace types;

namespace utils {

/**
 * @brief computeRouteCost
 * computes the cost of a given route
 * @param _route
 * @return the cost of _route
 */
double computeRouteCost(Route _route);

} // namespace utils
} // namespace routing
} // namespace mobility
} // namespace LIMoSim

#endif // ROUTINGUTILS_H
