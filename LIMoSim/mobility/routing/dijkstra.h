#ifndef LIMOSIM_DIJKSTRA_H
#define LIMOSIM_DIJKSTRA_H

#include <map>
#include <vector>
#include <deque>
#include "routable.h"

namespace LIMoSim
{

class Dijkstra
{
public:
    Dijkstra();

    void setGraph(std::vector<Routable*> &_graph);
    std::vector<Routable*> computeShortestPath(Routable *_start, Routable *_destination);

private:
    void init(Routable *_start);
    void updateDistance(Routable *_u, Routable *_v);
    Routable* getNearestNode();
    void removeNode(Routable *_node);
    int getNodeIndex(Routable *_node);

private:
    std::vector<Routable*> m_q;
    std::map<Routable*,double> m_distances;
    std::map<Routable*,Routable*> m_predecessors;
};

}

#endif // LIMOSIM_DIJKSTRA_H
