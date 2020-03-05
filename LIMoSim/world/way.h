#ifndef LIMOSIM_WAY_H
#define LIMOSIM_WAY_H

#include "worldobject.h"
#include "node.h"

namespace LIMoSim
{

class Way : public WorldObject
{
public:
    Way(const std::string &_id, const std::string &_type);

    virtual void addNode(Node *_node);
    Node* getStartNode();
    Node* getEndNode();
    bool isVertex(Node *_node);

    bool isClosed();

    //
    const std::vector<Node*>& getNodes();

    // WorldObject
    std::string toString();

protected:
    std::vector<Node*> m_nodes;
};

}


#endif // LIMOSIM_WAY_H
