#include "osmparser.h"
#include <iostream>
#include "wgs84.h"

namespace LIMoSim
{

OSMParser::OSMParser() :
    p_world(0),
    m_system(COORDINATE_SYSTEM::CARTESIAN),
    m_buildings(false)
{
    p_world = World::getInstance();
}


/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void OSMParser::parse(DOMElement *_document, bool _buildings)
{
    m_buildings = _buildings;

    std::string generator = _document->getAttribute("generator").toString();
    if(generator!="LIMoSim")
        m_system = COORDINATE_SYSTEM::WGS84;

    for(unsigned int i=0; i<_document->childNodes.size(); i++)
    {
        DOMElement *element = _document->childNodes.at(i)->toElement();
        std::string type = element->tagName;

        if(type=="node")
            parseNode(element);
        else if(type=="way")
            parseWay(element);
    }
}

void OSMParser::parseNode(DOMElement *_node)
{
    std::string id = _node->getAttribute("id").toString();
    Vector3d position(_node->getAttribute("x").toDouble(), _node->getAttribute("y").toDouble());
    Node *node = p_world->createNode(id, position);

    std::map<std::string, Variant> tags = readTags(_node);
    if(m_system==COORDINATE_SYSTEM::WGS84)
    {
        Vector3d position(_node->getAttribute("lon").toDouble(), _node->getAttribute("lat").toDouble());
        Vector3d origin = p_world->getReference();
        if(origin.norm()==0)
        {
            p_world->setReference(position);
            origin = position;
        }

        Vector3d cartesian = WGS84::computeOffset(position, origin);
        node->setPosition(cartesian);
    }

    if(tags.count("highway")>0)
    {
        std::string type = tags["highway"].toString();
        if(type=="traffic_signals")
        {
            //std::cout << node->toString() << "\t" << tags["highway"].toString() << std::endl;
        }
    }
}

void OSMParser::parseWay(DOMElement *_way)
{
    std::string id = _way->getAttribute("id").toString();
    std::vector<std::string> nodes;
    std::map<std::string, Variant> tags = readTags(_way);

    // read all nodes
    for(unsigned int i=0; i<_way->childNodes.size(); i++)
    {
        DOMElement *child = _way->childNodes.at(i)->toElement();
        if(child->tagName=="nd")
        {
            std::string node = child->getAttribute("ref").toString();
            nodes.push_back(node);
        }
    }

    std::map<std::string, int> roadTypes;
    roadTypes["motorway"] = ROAD_TYPE::MOTORWAY;
    roadTypes["motorway_link"] = ROAD_TYPE::MOTORWAY_LINK;
    roadTypes["motorway_junction"] = ROAD_TYPE::MOTORWAY_JUNCTION;
    roadTypes["trunk"] = ROAD_TYPE::TRUNK;
    roadTypes["trunk_link"] = ROAD_TYPE::TRUNK_LINK;
    roadTypes["primary"] = ROAD_TYPE::PRIMARY;
    roadTypes["secondary"] = ROAD_TYPE::SECONDARY;
    roadTypes["tertiary"] = ROAD_TYPE::TERTIARY;
    roadTypes["residential"] = ROAD_TYPE::RESIDENTIAL;
    //roadTypes["service"] = ROAD_TYPE::SERVICE;

    //
    Way *way = 0;
    std::string typeKey = tags["highway"].toString();
    if(roadTypes.count(typeKey)>0)
        way = parseRoad(id, roadTypes[typeKey], tags);
    else
    {
        if(tags.count("building")>0 && m_buildings)
            way = parseBuilding(id, tags);

        // areas
        if(tags.count("natural")>0)
            way = parseArea(id, tags["natural"].toString());
        else if(tags.count("leisure")>0)
            way = parseArea(id, tags["leisure"].toString());
        else if(tags.count("landuse")>0)
            way = parseArea(id, tags["landuse"].toString());
    }

    if(way) // once the way type is known, add the nodes
    {
        for(unsigned int i=0; i<nodes.size(); i++)
        {
            Node *node = p_world->getNodeById(nodes.at(i));
            way->addNode(node);

            if(way->getType()=="Building")
                p_world->updateBox(node->getPosition());
        }
    }
}

Road* OSMParser::parseRoad(const std::string &_id, int _type, std::map<std::string, Variant> &_tags)
{
    Road *road = p_world->createRoad(_id);
    std::map<std::string, Variant>::iterator it;

    bool oneway = false;
    if(_tags.count("oneway")>0 && _tags["oneway"].toString()=="yes")
        oneway = true;
    road->setType(_type, oneway);

    for(it=_tags.begin(); it!=_tags.end(); it++)
    {
        std::string key = it->first;
        Variant value = it->second;

        if(key=="lanes:forward")
            road->setForwardLanes(value.toInt());
        else if(key=="lanes:backward")
            road->setBackwardLanes(value.toInt());
        else if(key=="maxspeed")
            road->setMaxSpeed(value.toDouble());
        else if(key=="name")
            road->setName(value.toString());
    }

    road->setForwardLanes(1);
    road->setBackwardLanes(1);


    return road;
}

Building* OSMParser::parseBuilding(const std::string &_id, std::map<std::string, Variant> &_tags)
{
    Building *building = p_world->createBuilding(_id);

    std::map<std::string, Variant>::iterator it;
    for(it=_tags.begin(); it!=_tags.end(); it++)
    {
        std::string key = it->first;
        Variant value = it->second;

        if(key=="name")
            building->setName(value.toString());
        else if(key=="height")
            building->setHeight(value.toDouble());
        else if(key=="building:levels")
            building->setHeight(value.toDouble() * 3.0);
    }


    // pseudo-random height
    int s = _id.size();
    std::string id = _id.substr(s-2, s-1);
    double v = -0.5 + atof(id.c_str())/99.0;
    double delta = 10 * 2;
    double height = 20 + v * delta;
    building->setHeight(height);

    return building;
}

Area* OSMParser::parseArea(const std::string &_id, const std::string &_type)
{
    Area *area = p_world->createArea(_id);
    area->setAreaType(_type);

    return area;
}

std::map<std::string, Variant> OSMParser::readTags(DOMElement *_element)
{
    std::map<std::string, Variant> tags;

    for(unsigned int i=0; i<_element->childNodes.size(); i++)
    {
        DOMElement *child = _element->childNodes.at(i)->toElement();
        if(child->tagName=="tag")
        {
            std::string key = child->getAttribute("k").toString();
            Variant value = child->getAttribute("v");
            tags[key] = value;
        }
    }

    return tags;
}

std::string OSMParser::toString()
{
    DOMElement *document = new DOMElement("osm");
    document->setAttribute("generator", Variant("LIMoSim"));

    // add the nodes
    std::map<std::string, Node*> nodes = p_world->getNodes();
    std::map<std::string, Node*>::iterator it;
    for(it=nodes.begin(); it!=nodes.end(); it++)
    {
        document->appendChild(OSMNodeEntry::serialize(it->second));
    }

    // add the roads
    std::map<std::string, Road*> roads = p_world->getRoads();
    std::map<std::string, Road*>::iterator r;
    for(r=roads.begin(); r!=roads.end(); r++)
    {
        document->appendChild(OSMWayEntry::serializeRoad(r->second));
    }

    // TODO: add the buildings
    std::map<std::string, Building*> buildings = p_world->getBuildings();
    std::map<std::string, Building*>::iterator b;
    for(b=buildings.begin(); b!=buildings.end(); b++)
    {
        document->appendChild(OSMWayEntry::serializeBuilding(b->second));
    }

    std::string data = document->toString();
    delete document;

    return data;
}

}
