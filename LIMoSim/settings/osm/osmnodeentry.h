#ifndef LIMOSIM_OSMNODEENTRY_H
#define LIMOSIM_OSMNODEENTRY_H

#include "osmentry.h"
#include "LIMoSim/world/node.h"

namespace LIMoSim
{

class OSMNodeEntry
{
public:
    OSMNodeEntry();

    static DOMElement* serialize(Node *_node);
};

}

#endif // LIMOSIM_OSMNODEENTRY_H
