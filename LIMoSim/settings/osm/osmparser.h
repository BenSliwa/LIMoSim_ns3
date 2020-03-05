#ifndef LIMOSIM_OSMPARSER_H
#define LIMOSIM_OSMPARSER_H

#include "LIMoSim/settings/domelement.h"
#include "LIMoSim/world/world.h"
#include "osmnodeentry.h"
#include "osmwayentry.h"

namespace LIMoSim
{

namespace COORDINATE_SYSTEM
{
    enum{
        WGS84,
        CARTESIAN
    };
}

class OSMParser
{
public:
    OSMParser();

    void parse(DOMElement *_document, bool _buildings = false);
    void parseNode(DOMElement *_node);
    void parseWay(DOMElement *_way);

    Road* parseRoad(const std::string &_id, int _type, std::map<std::string, Variant> &_tags);
    Building* parseBuilding(const std::string &_id, std::map<std::string, Variant> &_tags);
    Area* parseArea(const std::string &_id, const std::string &_type);

    std::map<std::string, Variant> readTags(DOMElement *_element);

    std::string toString();


private:
    World *p_world;
    int m_system;
    bool m_buildings;
};

}


#endif // LIMOSIM_OSMPARSER_H
