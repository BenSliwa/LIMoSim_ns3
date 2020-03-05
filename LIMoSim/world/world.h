#ifndef LIMOSIM_WORLD_H
#define LIMOSIM_WORLD_H

#include "node.h"
#include "LIMoSim/world/road/road.h"
#include "LIMoSim/world/road/intersection.h"
#include "LIMoSim/world/road/gate.h"
#include "building.h"
#include "area.h"
#include <map>

namespace LIMoSim
{

class World
{
public:
    World();
    ~World();
    static World* getInstance();

    // initialization
    void loadMap(const std::string &_path);
    std::vector<Road*> filterRoads(const std::vector<Way*> &_ways);
    void filterNodes();
    void initIntersections();
    void linkIntersections();
    void initLanes();
    void updateBox(const Vector3d &_point);

    //
    void mergeLanes(RoadSegment *_from, RoadSegment *_to, int _fromType, int _toType);

    //

    std::vector<Routable*> buildRoutingGraph();

    // creation
    Node* createNode(const std::string &_id, const Vector3d &_position);
    Road* createRoad(const std::string &_id);
    Intersection* createIntersection(Node *_node);
    Gate* createGate(Endpoint *_endpoint);
    Building* createBuilding(const std::string &_id);
    Area* createArea(const std::string &_id);

    // destruction
    void clearNodes();
    void clearRoads();

    // accessors
    void setReference(const Vector3d &_origin);
    Vector3d getReference();
    Vector3d getBoxMin();
    Vector3d getBoxMax();
    Node* getNodeById(const std::string &_id);
    Road* getRoadById(const std::string &_id);
    Intersection* getIntersection(Node *_node);
    const std::map<std::string, Node*>& getNodes();
    const std::map<std::string, Road*>& getRoads();
    const std::map<Node*, Intersection*>& getIntersections();
    const std::map<std::string, Building*>& getBuildings();
    const std::map<std::string, Area*>& getAreas();

private:
    std::map<std::string, Node*> m_nodes;
    std::map<std::string, Road*> m_roads;
    std::map<Node*, Intersection*> m_intersections;
    std::map<std::string, Building*> m_buildings;
    std::map<std::string, Area*> m_areas;

    Vector3d m_reference;

    Vector3d m_boxMin;
    Vector3d m_boxMax;
};

}

#endif // LIMOSIM_WORLD_H
