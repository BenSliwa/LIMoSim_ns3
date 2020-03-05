#ifndef LIMOSIM_ROUTABLE_H
#define LIMOSIM_ROUTABLE_H

#include <sstream>
#include <map>

namespace LIMoSim
{

class Routable
{
public:
    Routable();

    //
    void addNeighbor(Routable *_node, double _cost);
    bool hasNeighbor(Routable *_node);
    double getCost(Routable *_node);

    const std::map<Routable*, double>& getNeighbors();

    //
    virtual std::string toString() = 0;

protected:
    std::map<Routable*, double> m_neigbors;
};

}


#endif // LIMOSIM_ROUTABLE_H
