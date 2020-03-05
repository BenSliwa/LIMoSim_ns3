#include "world.h"
#include "LIMoSim/settings/osm/osmparser.h"
#include "LIMoSim/settings/xmlparser.h"
#include <algorithm>

namespace LIMoSim
{

World::World()
{
    double max = std::numeric_limits<double>::max();
    m_boxMin = Vector3d(max, max, max);
    m_boxMax = Vector3d(-max, -max, -max);
}

World::~World()
{
    clearRoads();
    clearNodes();
}

World* World::getInstance()
{
    static World instance;
    return &instance;
}


/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void World::loadMap(const std::string &_path)
{
    XMLParser xml;
    DOMElement *map = xml.parse(_path);
    if(map)
    {
        std::cout << "parse OSM " << _path << std::endl;
        OSMParser osm;
        osm.parse(map, true);

        //FileHandler::write(osm.toString(), _path + ".limo");
    }
}

std::vector<Road*> World::filterRoads(const std::vector<Way*> &_ways)
{
    std::vector<Road*> roads;
    for(unsigned int i=0; i<_ways.size(); i++)
    {
        Road *road = dynamic_cast<Road*>(_ways.at(i));
        if(road)
            roads.push_back(road);
    }

    return roads;
}

void World::filterNodes()
{
    std::map<std::string, Node*>::iterator it;
    for(it=m_nodes.begin(); it!=m_nodes.end();)
    {
        Node *node = it->second;
        if(node->getWays().size()==0)
        {
            m_nodes.erase(it++);
            delete node;
        }
        else
            it++;

    }
}

void World::initIntersections()
{
    // assumption here: all ways have been set up already

    // TODO: clear intersections

    // for all nodes
    std::map<std::string, Node*>::iterator it;
    for(it=m_nodes.begin(); it!=m_nodes.end(); it++)
    {
        Node *node = it->second;
        std::vector<Road*> roads = filterRoads(node->getWays());
        if(roads.size()==2 && roads.at(0)->isVertex(node) && roads.at(1)->isVertex(node))
        {
            // the node connects multiple ways
            Road *road0 = roads.at(0);
            Road *road1 = roads.at(1);

            // TODO: determine the respective road segments
            RoadSegment *from = node==road0->getFirstSegment()->getFromEndpoint()->getNode() ? road0->getFirstSegment() : road0->getLastSegment();
            RoadSegment *to = node==road1->getFirstSegment()->getFromEndpoint()->getNode() ? road1->getFirstSegment() : road1->getLastSegment();


            // determine connection type and merge the lanes for both directions
           int type0 = road0->getEndNode()==node ? LANE_TYPE::FORWARD : LANE_TYPE::BACKWARD;
           int type1 = road1->getStartNode()==node ? LANE_TYPE::FORWARD : LANE_TYPE::BACKWARD;
           mergeLanes(from, to, type0, type1);

           type0 = road0->getEndNode()==node ? LANE_TYPE::BACKWARD : LANE_TYPE::FORWARD;
           type1 = road1->getStartNode()==node ? LANE_TYPE::BACKWARD : LANE_TYPE::FORWARD;
           mergeLanes(to, from, type1, type0);


         //   type0 = road0->getEndNode()==node ? LANE_TYPE::BACKWARD : LANE_TYPE::FORWARD;
         //   type1 = road1->getStartNode()==node ? LANE_TYPE::BACKWARD : LANE_TYPE::FORWARD;
           // mergeLanes(road1, road0, 0, 0);

            //mergeLanes(road0, road1, type0, type1);
           // mergeLanes(road1, road0, type0, type1);

           // std::cout << node->getId() <<": connecting " << road0->getId() << " to " << road1->getId() << std::endl;
        }
        else if(roads.size()==1 && !roads.at(0)->isVertex(node))
        {
            // the node connects multiple segments of the same road - nothing to do here
        }
        else if(roads.size()==0)
        {
            // the node is not connected to a road at all
        }
        else
        {
            // the node is an intersection
            Intersection *intersection = createIntersection(node);
            intersection->initialize(roads);
        }
    }

    //
}

void World::linkIntersections()
{
    int numGates = 0;
    std::map<Node*, Intersection*>::iterator it;
    for(it=m_intersections.begin(); it!=m_intersections.end(); it++)
    {
        Intersection *intersection = it->second;
        intersection->exploreDestinations();

        numGates += intersection->getOutGates().size();
    }

    std::cout << "found routing nodes: " << m_intersections.size() << " / " << numGates << std::endl;

}

void World::initLanes()
{
    // assumption here: all roads have been set up and connected
}

void World::updateBox(const Vector3d &_point)
{
    if(_point.x<m_boxMin.x)
        m_boxMin.x = _point.x;
    if(_point.y<m_boxMin.y)
        m_boxMin.y = _point.y;
    if(_point.z<m_boxMin.z)
        m_boxMin.z = _point.z;

    if(_point.x>m_boxMax.x)
        m_boxMax.x = _point.x;
    if(_point.y>m_boxMax.y)
        m_boxMax.y = _point.y;
    if(_point.z>m_boxMax.z)
        m_boxMax.z = _point.z;
}

void World::mergeLanes(RoadSegment *_from, RoadSegment *_to, int _fromType, int _toType)
{
    //std::cout << "merging " << _from->getRoad()->getId() << " to " << _to->getRoad()->getId() << "\t" << _fromType << "/" << _toType << std::endl;

    std::vector<LaneSegment*> from = _from->getLanes(_fromType);
    std::vector<LaneSegment*> to = _to->getLanes(_toType);
    unsigned int s = std::min(from.size(), to.size());
    for(unsigned int i=0; i<s; i++)
    {
        LaneSegment *fromLane = from.at(i);
        LaneSegment *toLane = to.at(i);

        fromLane->setNext(toLane);
    }
}


std::vector<Routable*> World::buildRoutingGraph()
{
    std::vector<Routable*> graph;
    std::map<Node*, Intersection*>::iterator it;
    for(it=m_intersections.begin(); it!=m_intersections.end(); it++)
    {
        Intersection *intersection = it->second;
        std::map<RoadSegment*, Gate*> gates = intersection->getOutGates();
        std::map<RoadSegment*, Gate*>::iterator g;
        for(g=gates.begin(); g!=gates.end(); g++)
            graph.push_back(g->second);
    }

    return graph;
}


/*************************************
 *          METHODS: CREATION        *
 ************************************/

Node* World::createNode(const std::string &_id, const Vector3d &_position)
{
    Node *node = new Node(_id, _position);
    m_nodes[_id] = node;


    return node;
}

Road* World::createRoad(const std::string &_id)
{
    Road *road = new Road(_id);
    m_roads[_id] = road;

    return road;
}

Intersection* World::createIntersection(Node *_node)
{
    Intersection *intersection = new Intersection(_node);
    m_intersections[_node] = intersection;

    return intersection;
}

Gate* World::createGate(Endpoint *_endpoint)
{
    Gate *gate = new Gate(_endpoint);

    // TODO: add to gate map

    return gate;
}

Building* World::createBuilding(const std::string &_id)
{
    Building *building = new Building(_id);
    m_buildings[_id] = building;

    return building;
}

Area* World::createArea(const std::string &_id)
{
    Area *area = new Area(_id);
    m_areas[_id] = area;

    return area;
}

/*************************************
 *        METHODS: DESTRUCTION       *
 ************************************/

void World::clearNodes()
{
    std::map<std::string, Node*>::iterator it;
    for(it=m_nodes.begin(); it!=m_nodes.end(); it++)
    {
        Node *node = it->second;
        delete node;
    }
    m_nodes.clear();
}

void World::clearRoads()
{
    std::map<std::string, Road*>::iterator it;
    for(it=m_roads.begin(); it!=m_roads.end(); it++)
    {
        Road *road = it->second;
        delete road;
    }
    m_roads.clear();
}


/*************************************
 *              ACCESSORS            *
 ************************************/

void World::setReference(const Vector3d &_origin)
{
    m_reference = _origin;
}

Vector3d World::getReference()
{
    return m_reference;
}

Vector3d World::getBoxMin()
{
    return m_boxMin;
}

Vector3d World::getBoxMax()
{
    return m_boxMax;
}

Node* World::getNodeById(const std::string &_id)
{
    if(m_nodes.count(_id)>0)
        return m_nodes[_id];
    return 0;
}

Road* World::getRoadById(const std::string &_id)
{
    if(m_roads.count(_id)>0)
        return m_roads[_id];
    return 0;
}

Intersection* World::getIntersection(Node *_node)
{
    if(m_intersections.count(_node)>0)
        return m_intersections[_node];
    return 0;
}

const std::map<std::string, Node*>& World::getNodes()
{
    return m_nodes;
}

const std::map<std::string, Road*>& World::getRoads()
{
    return m_roads;
}

const std::map<Node*, Intersection*>& World::getIntersections()
{
    return m_intersections;
}

const std::map<std::string, Building*>& World::getBuildings()
{
    return m_buildings;
}

const std::map<std::string, Area*>& World::getAreas()
{
    return m_areas;
}

}
