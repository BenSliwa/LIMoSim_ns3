#ifndef LIMOSIM_ROADSEGMENT_H
#define LIMOSIM_ROADSEGMENT_H

#include "LIMoSim/world/worldobject.h"
#include "LIMoSim/world/node.h"
#include "lanesegment.h"
#include "endpoint.h"

/* Segments are connected to their neighbors on the same way upon creation
 *
 *
 */

namespace LIMoSim
{
class Road;

class RoadSegment : public WorldObject
{
public:
    RoadSegment(Node *_from, Node *_to, int _index, Road *_road);
    virtual ~RoadSegment();

    //
    void initializeEndpointPositions();


    // RoadSegment
    void connect(RoadSegment *_segment, Node *_node);
    void disconnect(RoadSegment *_segment, Node *_node);
    Endpoint* getFromEndpoint();
    Endpoint* getToEndpoint();
    std::vector<LaneSegment*> getForwardLanes();

    //
    Node* getOtherNode(Node *_node);


    //
    double getLength();
    Vector3d getDirection();

    //
    void setIndex(int _index);
    int getIndex();
    Road* getRoad();
    const std::vector<LaneSegment*>& getLanes(int _type);
    LaneSegment* getLane(int _type, int _index);

private:
    int m_index;
    Road *p_road;
    std::vector<LaneSegment*> m_forwardLanes;
    std::vector<LaneSegment*> m_backwardLanes;

    Endpoint m_from;
    Endpoint m_to;
};

}

#endif // LIMOSIM_ROADSEGMENT_H
