#include "roadsegment.h"
#include "road.h"

namespace LIMoSim
{

RoadSegment::RoadSegment(Node *_from, Node *_to, int _index, Road *_road) :
    WorldObject("RoadSegment"),
    m_index(_index),
    p_road(_road)
{
    m_from.setNode(_from);
    m_from.setOwner(this);
    m_to.setNode(_to);
    m_to.setOwner(this);

    initializeEndpointPositions();

    Vector3d dir = getDirection();
    double laneWidth_m = p_road->getLaneWidth();
    for(int i=0; i<p_road->getForwardLanes(); i++)
    {
        LaneSegment *lane = new LaneSegment(this, LANE_TYPE::FORWARD, i);
        m_forwardLanes.push_back(lane);

        Vector3d start = m_from.center + dir.rotateRight() * (laneWidth_m * i + laneWidth_m/2);
        Vector3d end = m_to.center + dir.rotateRight() * (laneWidth_m * i + laneWidth_m/2);

        lane->setStart(start);
        lane->setEnd(end);
    }
    for(int i=0; i<p_road->getBackwardLanes(); i++)
    {
        LaneSegment *lane = new LaneSegment(this, LANE_TYPE::BACKWARD, i);
        m_backwardLanes.push_back(lane);

        Vector3d end = m_from.center + dir.rotateLeft() * (laneWidth_m * i + laneWidth_m/2);
        Vector3d start = m_to.center + dir.rotateLeft() * (laneWidth_m * i + laneWidth_m/2);

        lane->setStart(start);
        lane->setEnd(end);
    }
}

RoadSegment::~RoadSegment()
{
    for(unsigned int i=0; i<m_forwardLanes.size(); i++)
        delete m_forwardLanes.at(i);
    for(unsigned int i=0; i<m_backwardLanes.size(); i++)
        delete m_backwardLanes.at(i);
}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void RoadSegment::initializeEndpointPositions()
{
    int forwardLanes = p_road->getForwardLanes();
    int backwardLanes = p_road->getBackwardLanes();
    double laneWidth_m = p_road->getLaneWidth();

    Vector3d dir = getDirection();
    m_from.leftVertex = m_from.center + dir.rotateLeft() * laneWidth_m * backwardLanes;
    m_from.rightVertex = m_from.center + dir.rotateRight() * laneWidth_m * forwardLanes;
    m_to.leftVertex = m_to.center + dir.rotateLeft() * laneWidth_m * backwardLanes;
    m_to.rightVertex = m_to.center + dir.rotateRight() * laneWidth_m * forwardLanes;
}

void RoadSegment::connect(RoadSegment *_segment, Node *_node)
{
    // TODO:
}

void RoadSegment::disconnect(RoadSegment *_segment, Node *_node)
{
    // TODO:
}

Endpoint* RoadSegment::getFromEndpoint()
{
    return &m_from;
}

Endpoint* RoadSegment::getToEndpoint()
{
    return &m_to;
}

std::vector<LaneSegment *> RoadSegment::getForwardLanes()
{
    return m_forwardLanes;
}

Node* RoadSegment::getOtherNode(Node *_node)
{
    if(m_to.getNode()==_node)
        return m_from.getNode();
    else if(m_from.getNode()==_node)
        return m_to.getNode();
    return 0;
}

double RoadSegment::getLength()
{
    return (m_from.getNode()->getPosition()-m_to.getNode()->getPosition()).norm();
}

Vector3d RoadSegment::getDirection()
{
    return (m_to.getNode()->getPosition() - m_from.getNode()->getPosition()).normed();
}

void RoadSegment::setIndex(int _index)
{
    m_index = _index;
}

int RoadSegment::getIndex()
{
    return m_index;
}

Road* RoadSegment::getRoad()
{
    return p_road;
}

const std::vector<LaneSegment*>& RoadSegment::getLanes(int _type)
{
    if(_type==LANE_TYPE::FORWARD)
        return m_forwardLanes;
    return m_backwardLanes;
}

LaneSegment* RoadSegment::getLane(int _type, int _index)
{
    if(_type==LANE_TYPE::FORWARD && m_forwardLanes.size()>_index)
        return m_forwardLanes.at(_index);
    else if(_type==LANE_TYPE::BACKWARD && m_backwardLanes.size()>_index)
        return m_backwardLanes.at(_index);
    return 0;
}

}
