#ifndef LIMOSIM_NODE_H
#define LIMOSIM_NODE_H

#include "worldobject.h"

/* Nodes know:
 * - the ways they are on
 * - the RoadSensors and TrafficLights related to them
 * - the Endpoints that refer to them
 *
 *
 */

namespace LIMoSim
{
class Way;

class Node : public WorldObject
{
public:
    Node(const std::string &_id, const Vector3d &_position);
    virtual ~Node();

    // Ways
    void registerWay(Way *_way);
    void deregisterWay(Way *_way);
    const std::vector<Way*>& getWays();


    //
    void setPosition(const Vector3d &_position);
    Vector3d getPosition();

    // WorldObject
    std::string toString();

private:
    Vector3d m_position;
    std::vector<Way*> m_ways;
};

}

#endif // NODE_H
