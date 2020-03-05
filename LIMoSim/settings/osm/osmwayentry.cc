#include "osmwayentry.h"


namespace LIMoSim
{

OSMWayEntry::OSMWayEntry()
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/


DOMElement* OSMWayEntry::serializeWay(Way *_way)
{
    DOMElement *element = new DOMElement("way");
    element->setAttribute("id", _way->getId());

    std::vector<Node*> nodes = _way->getNodes();
    for(unsigned int i=0; i<nodes.size(); i++)
    {
        Node *node = nodes.at(i);
        DOMElement *child = new DOMElement("nd");
        child->setAttribute("ref", node->getId());
        element->appendChild(child);
    }

    return element;
}

DOMElement* OSMWayEntry::serializeRoad(Road *_road)
{
    DOMElement *element = serializeWay(_road);

    std::map<int, std::string> typeKeys;
    typeKeys[ROAD_TYPE::MOTORWAY] = "motorway";
    typeKeys[ROAD_TYPE::MOTORWAY_LINK] = "motorway_link";
    typeKeys[ROAD_TYPE::MOTORWAY_JUNCTION] = "motorway_junction";
    typeKeys[ROAD_TYPE::TRUNK] = "trunk";
    typeKeys[ROAD_TYPE::TRUNK_LINK] = "trunk_link";
    typeKeys[ROAD_TYPE::PRIMARY] = "primary";
    typeKeys[ROAD_TYPE::SECONDARY] = "secondary";
    typeKeys[ROAD_TYPE::TERTIARY] = "tertiary";
    typeKeys[ROAD_TYPE::RESIDENTIAL] = "residential";
    typeKeys[ROAD_TYPE::SERVICE] = "service";

    addTag("highway", typeKeys[_road->getType()], element);
    addTag("name", _road->getName(), element);

    return element;
}

DOMElement* OSMWayEntry::serializeBuilding(Building *_building)
{
    DOMElement *element = serializeWay(_building);
    addTag("building", Variant("yes"), element);
    addTag("height", _building->getHeight(), element);
    if(_building->getName()!="")
        addTag("name", _building->getName(), element);


    return element;
}

}
