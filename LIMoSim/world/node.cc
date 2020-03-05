#include "node.h"
#include "way.h"

namespace LIMoSim
{

Node::Node(const std::string &_id, const Vector3d &_position) :
    WorldObject("Node", _id),
    m_position(_position)
{
}

Node::~Node()
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void Node::registerWay(Way *_way)
{
    // TODO: check if the way is already registererd
    m_ways.push_back(_way);
}

void Node::deregisterWay(Way *_way)
{
    // TODO:
}

const std::vector<Way*>& Node::getWays()
{
    return m_ways;
}

void Node::setPosition(const Vector3d &_position)
{
    m_position = _position;
}

Vector3d Node::getPosition()
{
    return m_position;
}

std::string Node::toString()
{
    return m_id;
}

}
