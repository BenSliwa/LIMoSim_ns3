#include "road.h"
#include "LIMoSim/world/world.h"

namespace LIMoSim
{

Road::Road(const std::string &_id) :
    Way(_id, "Road"),
    m_forwardLanes(1),
    m_backwardLanes(1),
    m_type(ROAD_TYPE::PRIMARY),
    m_maxSpeed_mps(50/3.6),
    m_laneWidth_m(3.5)
{
    setForwardLanes(1);
    setBackwardLanes(1);
}

Road::~Road()
{
    for(unsigned int i=0; i<m_segments.size(); i++)
        delete m_segments.at(i);
}


/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void Road::initLanes(unsigned int _forward, unsigned int _backward)
{
    // assumption here: the segment infrastructure has already been set up

    // TODO: clear lanes

    for(unsigned int i=0; i<_forward; i++)
    {
        //Lane *lane = new Lane(this);


        // TODO: if the road has a predecessor, the add the items to the lane
        // else create a new lane




        for(unsigned int j=0; j<m_segments.size(); j++)
        {
            RoadSegment *roadSegment = m_segments.at(i);
/*
            // TODO: compute the positions
            Position from;
            Position to;

            LaneSegment *segment = new LaneSegment(from, to, lane, roadSegment);
            lane->addSegment(segment);*/
        }
    }


    // TODO: set up backward lanes
}

void Road::initSegmentConnections()
{
    if(m_segments.size()>1)
    {
        for(unsigned int i=1; i<m_segments.size(); i++)
        {
            RoadSegment *previous = m_segments.at(i-1);
            RoadSegment *segment = m_segments.at(i);

            // TODO: connect
        }


        // check for loopsbacks
        if(getStartNode()==getEndNode())
        {
            // TODO: close the loop
        }
    }
}

bool Road::isSuperior(Road *_to)
{
    return m_type>_to->getType();
}

void Road::connect(Road *_road, Node *_node)
{
    // TODO: find the corresponding segments and connect them




    // respect the connection type


    /*
    if()
    {
        World::getInstance()->createLane(origin);
    }*/
}


void Road::disconnect(Road *_road, Node *_node)
{
    // TODO:
}

void Road::addNode(Node *_node)
{
    Node *endNode = getEndNode();
    Way::addNode(_node);

    //
    if(m_nodes.size()>1) // create a new road segment if the road already has nodes
    {
        RoadSegment *segment = new RoadSegment(endNode, _node, m_segments.size(), this);
        addSegment(segment);
    }
}

void Road::addSegment(RoadSegment *_segment)
{
    if(m_segments.size()>0) //  link it to its predecessor
    {
        RoadSegment *previous = getLastSegment();

        // TODO: connect the segment lanes
        if(previous)
        {
            std::vector<LaneSegment*> forwardLanes = _segment->getLanes(LANE_TYPE::FORWARD);
            std::vector<LaneSegment*> previousForwardLanes = previous->getLanes(LANE_TYPE::FORWARD);
            for(unsigned int i=0; i<forwardLanes.size(); i++)
                previousForwardLanes.at(i)->setNext(forwardLanes.at(i));

            std::vector<LaneSegment*> backwardLanes = _segment->getLanes(LANE_TYPE::BACKWARD);
            std::vector<LaneSegment*> previousBackwardLanes = previous->getLanes(LANE_TYPE::BACKWARD);
            for(unsigned int i=0; i<backwardLanes.size(); i++)
            {
               // previousBackwardLanes.at(i)->setNext(backwardLanes.at(i));
                backwardLanes.at(i)->setNext(previousBackwardLanes.at(i));
            }


        }


    }
    m_segments.push_back(_segment);
}

RoadSegment* Road::getFirstSegment()
{
    if(m_segments.size()>0)
        return m_segments.at(0);
    return 0;
}

RoadSegment* Road::getLastSegment()
{
    if(m_segments.size()>0)
        return m_segments.at(m_segments.size()-1);
    return 0;
}

RoadSegment* Road::getSegmentWithIndex(int _index)
{
    if(_index<m_segments.size())
        return m_segments.at(_index);
    return 0;
}

std::vector<RoadSegment*> Road::getSegmentsForNode(Node *_node)
{
    std::vector<RoadSegment*> segments;

    // TODO

    return segments;
}

const std::vector<RoadSegment*>& Road::getSegments()
{
    return m_segments;
}

/*************************************
 *          PROPERTY ACCESSORS       *
 ************************************/

void Road::setType(int _type, bool _oneway)
{
    m_type = _type;

    if(_type<=ROAD_TYPE::TRUNK_LINK)
    {
        setForwardLanes(2);
        setBackwardLanes(0);
    }
    else if(_oneway)
    {
        setForwardLanes(1);
        setBackwardLanes(0);
    }
}

void Road::setName(const std::string &_name)
{
    m_name = _name;
}

void Road::setMaxSpeed(double _speed_kmh)
{
    m_maxSpeed_mps = _speed_kmh / 3.6;
}

void Road::setForwardLanes(int _lanes)
{
    m_forwardLanes = _lanes;
}

void Road::setBackwardLanes(int _lanes)
{
    m_backwardLanes = _lanes;
}

void Road::setLaneWidth(double _laneWidth_m)
{
    m_laneWidth_m = _laneWidth_m;
}

int Road::getType()
{
    return m_type;
}

std::string Road::getName()
{
    return m_name;
}

double Road::getMaxSpeed()
{
    return m_maxSpeed_mps;
}

int Road::getForwardLanes()
{
    return m_forwardLanes;
}

int Road::getBackwardLanes()
{
    return m_backwardLanes;
}

double Road::getLaneWidth()
{
    return m_laneWidth_m;
}


/*************************************
 *       INHERITED: WorldObject      *
 ************************************/

std::string Road::toString()
{
    std::stringstream stream;
    stream << m_id << ": ";
    for(unsigned int i=0; i<m_segments.size(); i++)
    {
        stream << "Seg" << i;
        if(i<m_segments.size()-1)
            stream << " -> ";
    }
    return stream.str();
}

}

