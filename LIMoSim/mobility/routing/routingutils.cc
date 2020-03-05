#include "LIMoSim/mobility/routing/routingutils.h"

#include "LIMoSim/mobility/routing/routable.h"

namespace LIMoSim {
namespace mobility {
namespace routing {
namespace utils {

double computeRouteCost(Route _route)
{
    double cost = 0;
    for (size_t i = 0; i < _route.size() - 1 ; i++) {
        Routable *current = _route.at(i), *next = _route.at(i+1);
        cost += current->getCost(next);
    }
    return cost;
}



} // namespace utils
} // namespace routing
} // namespace mobility
} // namespace LIMoSim
