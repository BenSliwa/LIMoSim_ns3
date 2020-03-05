#ifndef LIMOSIM_ROADSENSOR_H
#define LIMOSIM_ROADSENSOR_H

/* RoadSensors measure traffic related information, e.g. traffic flow
 *
 *
 *
 *
 */

namespace LIMoSim
{
class Node;

class RoadSensor
{
public:
    RoadSensor(Node *_node);



private:
    Node *p_node;
};

}

#endif // ROADSENSOR_H
