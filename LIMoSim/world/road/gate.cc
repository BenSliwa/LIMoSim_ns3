#include "gate.h"
#include "road.h"
#include <sstream>

namespace LIMoSim
{

Gate::Gate(Endpoint *_endpoint) :
    Routable(),
    p_endpoint(_endpoint)
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

Endpoint* Gate::getEndpoint()
{
    return p_endpoint;
}

std::string Gate::toString()
{
    RoadSegment *segment = p_endpoint->getOwner();
    std::stringstream stream;
    stream << "N" << p_endpoint->getNode()->getId();
    stream << "_W" << segment->getRoad()->getId() << "_" << segment->getIndex();

    return stream.str();
}

}
