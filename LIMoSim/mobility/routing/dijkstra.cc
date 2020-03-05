#include "dijkstra.h"
#include <limits>
#include <iostream>

namespace LIMoSim
{

Dijkstra::Dijkstra()
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

std::vector<Routable*> Dijkstra::computeShortestPath(Routable *_start, Routable *_destination)
{
    init(_start);

    while(m_q.size()>0)
    {
        Routable *u = getNearestNode();
        if(!u) // no path exists
            break;
        removeNode(u);

        std::map<Routable*, double> neighbors = u->getNeighbors();
        std::map<Routable*, double>::iterator it;
        for(it=neighbors.begin(); it!=neighbors.end(); it++)
        {
            Routable *v = it->first;
            if(getNodeIndex(v)>-1) // v is in Q
                updateDistance(u, v);
        }
    }

    // create the routing path
    std::vector<Routable*> path;
    Routable *node = _destination;
    while(m_predecessors[node])
    {
        path.insert(path.begin(), node);
        node = m_predecessors[node];

    }
    path.insert(path.begin(), node);


//    std::cout << "Dijkstra::computeShortestPath\t";
//    for(int i=0; i<path.size(); i++)
//    {
//        std::cout << path.at(i)->toString() << " ";
//    }
//    std::cout << std::endl;

    return path;
}

void Dijkstra::setGraph(std::vector<Routable*> &_graph)
{
    m_q = _graph;
}

/*************************************
 *           PRIVATE METHODS         *
 ************************************/

void Dijkstra::init(Routable *_start)
{
    for(unsigned int i=0; i<m_q.size(); i++)
    {
        Routable *node = m_q.at(i);
        m_distances[node] = std::numeric_limits<double>::infinity();
        m_predecessors[node] = 0;
    }

    m_distances[_start] = 0;
}

void Dijkstra::updateDistance(Routable *_u, Routable *_v)
{
    double distance_m = m_distances[_u] + _u->getCost(_v);
    if(distance_m<m_distances[_v])
    {
        m_distances[_v] = distance_m;
        m_predecessors[_v] = _u;
    }
}

Routable* Dijkstra::getNearestNode()
{
    Routable *node = 0;
    double minDistance_m = std::numeric_limits<double>::infinity();

    for(unsigned int i=0; i<m_q.size(); i++)
    {
        Routable *currentNode = m_q.at(i);
        double distance_m = m_distances[currentNode];
        if(distance_m<minDistance_m)
        {
            node = currentNode;
            minDistance_m = distance_m;
        }
    }

    return node;
}

void Dijkstra::removeNode(Routable *_node)
{
    int index = getNodeIndex(_node);
    if(index>-1)
        m_q.erase(m_q.begin() + index);
}

int Dijkstra::getNodeIndex(Routable *_node)
{
    int index = -1;
    for(unsigned int i=0; i<m_q.size(); i++)
    {
        if(m_q.at(i)==_node)
        {
            index = i;
            break;
        }
    }

    return index;
}

}
