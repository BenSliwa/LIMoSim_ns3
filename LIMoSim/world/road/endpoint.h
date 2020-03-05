#ifndef LIMOSIM_ENDPOINT_H
#define LIMOSIM_ENDPOINT_H

#include "LIMoSim/world/node.h"

/* Connection Points for Intersections
 * Endpoints can only be connected to a single other endpoint
 * Endpoints store the position information of segments
 *
 */

namespace LIMoSim
{
class RoadSegment;

class Endpoint
{
public:
    Endpoint();

    void connect(Endpoint *_endpoint);
    void disconnect();

    //
    void setOwner(RoadSegment *_owner);
    void setNode(Node *_node);

    RoadSegment* getOwner();
    Node* getNode();


private:
    RoadSegment *p_owner;
    Node *p_node; // reference node
    Endpoint *p_connection;

public:
    Vector3d center;
    Vector3d leftVertex;
    Vector3d rightVertex;

};

typedef std::vector<Endpoint*> Endpoints;

}

#endif // ENDPOINT_H
