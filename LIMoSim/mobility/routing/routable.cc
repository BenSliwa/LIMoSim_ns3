#include "routable.h"

namespace LIMoSim
{

Routable::Routable()
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void Routable::addNeighbor(Routable *_node, double _cost)
{
    m_neigbors[_node] = _cost;
}

bool Routable::hasNeighbor(Routable *_node)
{
    return (m_neigbors.count(_node)>0);
}

double Routable::getCost(Routable *_node)
{
    return m_neigbors[_node];
}

const std::map<Routable*, double>& Routable::getNeighbors()
{
    return m_neigbors;
}

}
