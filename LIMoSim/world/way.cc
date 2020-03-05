#include "way.h"

namespace LIMoSim
{

Way::Way(const std::string &_id, const std::string &_type) :
    WorldObject(_type, _id)
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void Way::addNode(Node *_node)
{
    m_nodes.push_back(_node);
    _node->registerWay(this);
}

Node* Way::getStartNode()
{
    if(m_nodes.size()>0)
        return m_nodes.at(0);
    return 0;
}

Node* Way::getEndNode()
{
    if(m_nodes.size()>0)
        return m_nodes.at(m_nodes.size()-1);
    return 0;
}

bool Way::isVertex(Node *_node)
{
    if(m_nodes.size()>1 && (_node==getStartNode() || _node==getEndNode()))
        return true;
    return false;
}

bool Way::isClosed()
{
    if(m_nodes.size()>0 && (getEndNode()==getStartNode()))
        return true;
    return false;
}

const std::vector<Node*>& Way::getNodes()
{
    return m_nodes;
}

std::string Way::toString()
{
    std::stringstream stream;
    for(unsigned int i=0; i<m_nodes.size(); i++)
    {
        stream << m_nodes.at(i)->toString();
        if(i<m_nodes.size()-1)
            stream << " -> ";
    }
    return stream.str();
}

}
