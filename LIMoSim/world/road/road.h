#ifndef LIMOSIM_ROAD_H
#define LIMOSIM_ROAD_H

#include "LIMoSim/world/way.h"
#include "roadsegment.h"
#include "roaddefinitions.h"

namespace LIMoSim
{



class Road : public Way
{
public:
    Road(const std::string &_id);
    virtual ~Road();

    void initLanes(unsigned int _forward, unsigned int _backward);
    void initSegmentConnections();

    bool isSuperior(Road *_to);

    // Roads
    void connect(Road *_road, Node *_node);
    void disconnect(Road *_road, Node *_node);

    // Nodes
    virtual void addNode(Node *_node);

    // Segments
    void addSegment(RoadSegment *_segment);
    RoadSegment* getFirstSegment();
    RoadSegment* getLastSegment();
    RoadSegment* getSegmentWithIndex(int _index);
    std::vector<RoadSegment*> getSegmentsForNode(Node *_node);
    const std::vector<RoadSegment*>& getSegments();

    // property accessors
    void setType(int _type, bool _oneway);
    void setName(const std::string &_name);
    void setMaxSpeed(double _speed_kmh);
    void setForwardLanes(int _lanes);
    void setBackwardLanes(int _lanes);
    void setLaneWidth(double _laneWidth_m);
    int getType();
    std::string getName();
    double getMaxSpeed();
    int getForwardLanes();
    int getBackwardLanes();
    double getLaneWidth();

    // WorldObject
    std::string toString();

private:
    std::vector<RoadSegment*> m_segments;

    int m_forwardLanes;
    int m_backwardLanes;

    //
    int m_type;
    std::string m_name;
    double m_maxSpeed_mps;
    double m_laneWidth_m;
};

}


#endif // LIMOSIM_ROAD_H
