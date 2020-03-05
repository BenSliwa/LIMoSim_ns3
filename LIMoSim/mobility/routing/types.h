#ifndef TYPES_H
#define TYPES_H


#include <vector>

//#include "routable.h"

namespace LIMoSim {

class Intersection;
class Routable;
class Road;
class Gate;

namespace mobility {
namespace routing {
namespace types {


typedef std::vector<Routable*> Route;

} // namespace types
} // namespace routing
} // namespace mobility
} // namespace LIMoSim

#endif // TYPES_H
