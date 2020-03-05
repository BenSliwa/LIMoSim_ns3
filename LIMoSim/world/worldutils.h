#ifndef WORLDUTILS_H
#define WORLDUTILS_H

#include "vector3d.h"
#include <map>
#include <vector>
#include <memory>

#include "worldtypes.h"
#include "LIMoSim/utils/typedefs.h"

namespace LIMoSim {
double controlledDescent(double x1, double descentStart, double descentFinalValue, double descentStartValue, double value, bool symmetric = false);
Vector3d controlledDescent(double descentEnd, double descentStart, double descentFinalValue, double descentStartValue, Vector3d value);
/**
 * @brief normalizeAngle
 * brings all angles in range [0, 360]
 * @param _angle_deg
 * @return
 */
double normalizeAngle(double _angle_deg);
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));

}


class Building;
class Gate;
class Endpoint;
class Node;

namespace world {
namespace utils {

using namespace types;
using namespace LIMoSim::utils::typedefs;

typedef std::map<Building*, Gate*> BuildingToGate;
typedef std::map<Building*, Endpoint*> BuildingToEndpoint;
typedef std::vector<Building*> Buildings;

struct RandomBuildingSelection {
    StringVector buildingIds;
    std::vector<Endpoint*> buildingEndpoints;
};


Vector3d getBuildingPosition(Building* p_building);

/**
 * @brief getNextExplorationRoad
 * Get the other road which is linked to a given road
 * through a given node.
 * @param _road
 * @param _node the junction node. Must be an endpoint of the road
 * @return the road connected the given one through the given node.
 */
Road* getNextExplorationRoad(Road *_road, Node *_node);

/**
 * @brief getNextIntersectionForEndpoint
 * Retrieve the next intersection to be encountered
 * when starting from a given endpoint with a given
 * driving direction.
 * @param _endpoint starting point
 * @param _forward driving direction
 * @return
 */
IntersectionDiscovery getNextIntersectionForEndpoint(
        Endpoint *_endpoint,
        const bool &_forward = true);


size_t hashDeliveryList(StringVector _deliveryList);

bool isIntersecting(const Vector3d &_p1, const Vector3d &_p2, const Vector3d &_q1, const Vector3d &_q2);

Gate* nearestGateToBuilding(Building* p_building);
BuildingToGate nearestGateToBuilding(Buildings _buildings);
BuildingToEndpoint nearestEndpointToBuilding(Buildings _buildings);
Endpoint *nearestEndpointToBuilding(Building *_building);


RandomBuildingSelection selectRandomBuildings(uint _buildingCount, StringVector _excluded = {});


} // namespace utils
} // namespace world
} // namespace LIMoSim
#endif // WORLDUTILS_H
