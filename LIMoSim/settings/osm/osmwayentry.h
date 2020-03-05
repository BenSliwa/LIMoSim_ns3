#ifndef LIMOSIM_OSMWAYENTRY_H
#define LIMOSIM_OSMWAYENTRY_H

#include "osmentry.h"
#include "LIMoSim/world/road/road.h"
#include "LIMoSim/world/building.h"

namespace LIMoSim
{

class OSMWayEntry : public OSMEntry
{
public:
    OSMWayEntry();

    static DOMElement* serializeWay(Way *_way);
    static DOMElement* serializeRoad(Road *_road);
    static DOMElement* serializeBuilding(Building *_building);

};

}

#endif // LIMOSIM_OSMWAYENTRY_H
