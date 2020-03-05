#include "osmnodeentry.h"


namespace LIMoSim
{

OSMNodeEntry::OSMNodeEntry()
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

DOMElement* OSMNodeEntry::serialize(Node *_node)
{
    DOMElement *element = new DOMElement("node");
    element->setAttribute("id", _node->getId());

    Vector3d position = _node->getPosition();
    element->setAttribute("x", position.x);
    element->setAttribute("y", position.y);
    element->setAttribute("z", position.z);

    return element;
}

}
