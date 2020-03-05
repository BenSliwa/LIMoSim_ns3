#ifndef LIMOSIM_GATE_H
#define LIMOSIM_GATE_H

#include "LIMoSim/mobility/routing/routable.h"
#include "endpoint.h"
#include <map>

/* Gates are owned by Intersections and refer to an Endpoint
 * Gates are automatically created by the Intersections
 * Routing is performed on Gates
 * Gates can be connected to multiple other gates
 *
 *
 *
 *
 * TODO: should gates know their destinations?
 *
 */

namespace LIMoSim
{

class Gate : public Routable
{
public:
    Gate(Endpoint *_endpoint);

    //
    Endpoint* getEndpoint();

    //
    std::string toString();

private:
    Endpoint *p_endpoint;
};

}

#endif // LIMOSIM_GATE_H
