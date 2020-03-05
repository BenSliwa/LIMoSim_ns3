#include "samerandompath.h"
#include "LIMoSim/world/world.h"
#include "LIMoSim/mobility/routing/dijkstra.h"
#include "LIMoSim/world/road/roadutils.h"

namespace LIMoSim {

SameRandomPath::SameRandomPath(Car *_car):
    FollowPath (_car)
{

}

SameRandomPath::~SameRandomPath()
{

}




void SameRandomPath::initialize()
{
    std::cout << "SameRandomPath::initialize" << std::endl;

    static RoadPosition s_info = p_car->getRoadPosition();

    Gate *start = s_info.path.at(0);
    m_start = start;

    static Gate *destination = nullptr;
    while(destination==nullptr || destination==start)
        destination = computeRandomDestination();
    m_destination = destination;

    Dijkstra dijkstra;
    std::vector<Routable*> graph = World::getInstance()->buildRoutingGraph();
    dijkstra.setGraph(graph);
    std::vector<Routable*> path = dijkstra.computeShortestPath(start, destination);
    RoadPosition info = s_info;
    info.path.clear();
    for(unsigned int i=1; i<path.size(); i++)
    {
        Gate *gate = dynamic_cast<Gate*>(path.at(i));
        info.path.push_back(gate);
    }
    m_reverse = false;

    //
    p_car->setRoadPosition(info);
    std::cout << "SameRandomPath::initialize done" << std::endl;
}

void SameRandomPath::handleNodeReached(Node *_node)
{
    FollowPath::handleNodeReached(_node);
}

void SameRandomPath::handleGateReached(Gate *_gate, Intersection *_intersection, LaneSegment *_lane)
{
    RoadPosition info = p_car->getRoadPosition();

    std::cout << "SameRandomPath::handleGateReached " << _gate->toString() << " " << info.path.size() << std::endl;


    if(info.path.size()>0)
    {
        Gate *gate = info.path.at(0);
        RoadSegment *segment = _intersection->getOutSegmentForGate(gate);

        if(segment) // reached a gate of the path
        {
            //
            RoadSegment *segment = _intersection->getOutSegmentForGate(gate);
            info.path.erase(info.path.begin());
            info.laneSegment = getNextLaneSegment(segment, _intersection, _lane);
            info.offset_m = 0;
            p_car->setRoadPosition(info);
        }
        else // reached a gate that is not part of the path -> rerouting
        {
        }
    }
    if (info.path.size() <2 ){
        // When destination reached
        // generate reverse path
        m_reverse = !m_reverse;
        Dijkstra dijkstra;
        std::vector<Routable*> graph = World::getInstance()->buildRoutingGraph();
        dijkstra.setGraph(graph);
        std::vector<Routable*> path = m_reverse ?
                    dijkstra.computeShortestPath(m_destination, m_start) :
                    dijkstra.computeShortestPath(m_start, m_destination);
        for(unsigned int i=1; i<path.size(); i++)
        {
            Gate *gate = dynamic_cast<Gate*>(path.at(i));
            info.path.push_back(gate);
        }
        p_car->setRoadPosition(info);

    }
}

//N430054740_W4236558_4 N164317_W8136201_0 N160374_W30977200_11
Gate *SameRandomPath::computeRandomDestination()
{
    std::vector<Routable*> graph = World::getInstance()->buildRoutingGraph();
    int index = rand() * (long)(graph.size()) / RAND_MAX;

    Gate *gate = dynamic_cast<Gate*>(graph.at(index));

    return gate;
}

} // namespace LIMoSim

// N273109929_W38182250_0 N164513_W371621205_18 N273109929_W122585630_0 N32558270_W30836171_0 N477805_W337056872_1 N164317_W337055448_0 N52919181_W49429902_3 N50835065_W6298398_17 N50835073_W6298398_14
