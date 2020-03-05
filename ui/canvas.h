#ifndef LIMOSIM_CANVAS_H
#define LIMOSIM_CANVAS_H

#include "LIMoSim/world/world.h"
#include "LIMoSim/world/node.h"


namespace LIMoSim
{

class Canvas
{
public:
    Canvas();

    virtual void drawNode(Node *_node) = 0;
    virtual void drawRoadSegment(RoadSegment *_segment) = 0;
    virtual void drawBuilding(Building *_building) = 0;

protected:
    World *p_world;

};

}

#endif // LIMOSIM_CANVAS_H
