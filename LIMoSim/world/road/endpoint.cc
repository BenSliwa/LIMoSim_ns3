#include "endpoint.h"

namespace LIMoSim
{

Endpoint::Endpoint() :
    p_owner(0),
    p_node(0),
    p_connection(0)
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void Endpoint::connect(Endpoint *_endpoint)
{
    p_connection = _endpoint;
}

void Endpoint::disconnect()
{
    p_connection = 0;
}

void Endpoint::setOwner(RoadSegment *_owner)
{
    p_owner = _owner;
}

void Endpoint::setNode(Node *_node)
{
    p_node = _node;
    center = p_node->getPosition();
}

RoadSegment* Endpoint::getOwner()
{
    return p_owner;
}

Node* Endpoint::getNode()
{
    return p_node;
}

}
