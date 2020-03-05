#include "deterministicpath.h"


#include "LIMoSim/world/world.h"
#include "LIMoSim/mobility/routing/routable.h"
#include "LIMoSim/mobility/routing/dijkstra.h"
#include "LIMoSim/world/road/road.h"

namespace LIMoSim {

DeterministicPath::DeterministicPath(Car *_car):
    FollowPath (_car)
{
//    m_gatesData.push_back(GateInfo("273109929", "38182250", 0));
//    m_gatesData.push_back(GateInfo("164513", "371621205", 18));
//    m_gatesData.push_back(GateInfo("273109929", "38182250", 0));
//    m_gatesData.push_back(GateInfo("32558270", "371621207", 0));
//    m_gatesData.push_back(GateInfo("273109929", "122585630", 0));
//    m_gatesData.push_back(GateInfo("32558270", "30836171", 0));
//    m_gatesData.push_back(GateInfo("477805", "337056872", 1));
//    m_gatesData.push_back(GateInfo("164317", "337055448", 0));
//    m_gatesData.push_back(GateInfo("52919181", "49429902", 3));
//    m_gatesData.push_back(GateInfo("50835065", "6298398", 17));
//    m_gatesData.push_back(GateInfo("50835073", "6298398", 14));

//    for (GateInfo gi : m_gatesData) {
//        Node* node = World::getInstance()->getNodeById(gi.nodeId);
//        Road* road = World::getInstance()->getRoadById(gi.roadId);
//        RoadSegment* seg = road->getSegmentWithIndex(gi.segmentIndex);
//        Gate* gate = new Gate(seg->getToEndpoint());
//        m_path.push_back(gate);
//    }
//    World *world = World::getInstance();
//    std::string nodeId = "160374"; //"160385";
//    //    nodeId = "164317";
//    auto i = world->getIntersection(world->getNodeById(nodeId));
//    auto og = i->getOutGates();
//    auto gs = i->getOutGates();
//    for (auto entry : gs) {
//        std::cout << "Node id: " << nodeId;
//        RoadSegment * rs = entry.first;
//        std::cout << rs->getRoad()->getName() << " idx:" << rs->getIndex();
//    }
//    m_gatesData.clear();
//    m_gatesData.push_back(GateInfo("160385","30977200",0));
//    m_gatesData.push_back(GateInfo("164317", "4236558", 0));

}

DeterministicPath::~DeterministicPath()
{

}




void DeterministicPath::initialize()
{
    std::cout << "DeterministicPath::initialize" << std::endl;

    // construct path from gate info
    World *world = World::getInstance();
    for (GateInfo gi : m_gatesData) {
        Node* node = World::getInstance()->getNodeById(gi.nodeId);
        Road* road = World::getInstance()->getRoadById(gi.roadId);
        RoadSegment* seg = road->getSegmentWithIndex(gi.segmentIndex);
        auto i = world->getIntersection(world->getNodeById(gi.nodeId));
        Gate* gate =i->getOutGate(seg);
//        Gate* gate = new Gate(seg->getFromEndpoint());
        m_path.push_back(gate);
    }

    Dijkstra dijkstra;
    std::vector<Routable*> graph = World::getInstance()->buildRoutingGraph();

    Gate *start=nullptr, *end=nullptr;
    for(Routable *routable: graph) {
        if (routable->toString() == m_path.at(0)->toString()) {
            std::cout << "DeterministicPath::initialize start found" << std::endl;
            start = dynamic_cast<Gate*>(routable);
        }
        if (routable->toString() == m_path.at(1)->toString()) {
            std::cout << "DeterministicPath::initialize end found" << std::endl;
            end = dynamic_cast<Gate*>(routable);
        }
    }

    if (!start) {
        std::cerr << "DeterministicPath::initialize Routables for start not found" << std::endl;
        exit(1);
    }

    if (!end) {
        std::cerr << "DeterministicPath::initialize Routables for end not found" << std::endl;
        exit(1);
    }

    dijkstra.setGraph(graph);
    std::vector<Routable*> path = dijkstra.computeShortestPath(start, end);

    RoadPosition roadPosition;
    for(unsigned int i=0; i<path.size(); i++)
    {
        Gate *gate = dynamic_cast<Gate*>(path.at(i));
        roadPosition.path.push_back(gate);
    }

    Gate *gate = dynamic_cast<Gate*>(path.at(0));
    roadPosition.laneSegment = gate->getEndpoint()->getOwner()->getLanes(LANE_TYPE::BACKWARD).at(0);
    roadPosition.offset_m = 0;

    p_car->setRoadPosition(roadPosition);

    std::cout << "DeterministicPath::initialize done" << std::endl;
}

void DeterministicPath::handleNodeReached(Node *_node)
{
    FollowPath::handleNodeReached(_node);
}

void DeterministicPath::handleGateReached(Gate *_gate, Intersection *_intersection, LaneSegment *_lane)
{
    RoadPosition info = p_car->getRoadPosition();
    std::cout << "DeterministicPath::handleGateReached " << _gate->toString() << " " << info.path.size() << std::endl;

    if(info.path.size()>0) {
        Gate *gate = info.path.at(0);
        RoadSegment *segment = _intersection->getOutSegmentForGate(gate);

        if(segment) // reached a gate of the path
        {
            //
            info.path.erase(info.path.begin());
            info.laneSegment = getNextLaneSegment(segment, _intersection, _lane);
            info.offset_m = 0;
            p_car->setRoadPosition(info);
        } else {
//            std::cerr << "DeterministicPath::handleGateReached " << "next segment not found!" << std::endl;
//            std::cout << "DeterministicPath::handleGateReached " << "patching route ..." << std::endl;
//            Dijkstra dijkstra;
//            std::vector<Routable*> graph = World::getInstance()->buildRoutingGraph();
//            dijkstra.setGraph(graph);
//            std::vector<Routable*> routePatch = dijkstra.computeShortestPath(_gate, gate);
//            std::vector<Gate*> patchedPath;
//            for (auto r: routePatch) {
//                patchedPath.push_back(dynamic_cast<Gate*>(r));
//            }
//            patchedPath.insert(patchedPath.end(), info.path.begin(), info.path.end());
//            info.laneSegment = gate->getEndpoint()->getOwner()->getLanes(LANE_TYPE::FORWARD).at(0);
//            info.offset_m = 0;
//            p_car->setRoadPosition(info);

            std::cerr << "DeterministicPath::handleGateReached " << "next segment not found!" << std::endl;
            Simulation::getInstance()->stop();
//            exit(122);
            // Try rerouting the truck back to the next gate in path
//            std::cout << "Rerouting ..." << std::endl;

//            Dijkstra dijkstra;
//            std::vector<Routable*> rg = World::getInstance()->buildRoutingGraph();
//            dijkstra.setGraph(rg);
//            std::vector<Routable*> route = dijkstra.computeShortestPath(_gate,gate);
//            if (route.size() == 0) {
//                std::cerr << "Rerouting failed." << std::endl;
//                exit(122);
//            } else {
//                std::vector<Gate*> reroutingPath;
//                for(auto routable : route) {
//                    reroutingPath.push_back(dynamic_cast<Gate*>(routable));
//                }
//                info.path.insert(info.path.begin(),reroutingPath.begin(), reroutingPath.end());
//                //_lane->getSegment();
//                RoadSegment *segment_rerouted = _intersection->getOutSegmentForGate(info.path.at(0));
//                if(segment_rerouted) // reached a gate of the path
//                {
//                    //
//                    info.path.erase(info.path.begin());
//                    info.laneSegment = getNextLaneSegment(segment_rerouted, _intersection, _lane);
//                    info.offset_m = 0;
//                    p_car->setRoadPosition(info);
//                } else {
//                    std::cerr << "DeterministicPath::handleGateReached " << "next rerouting segment not found!" << std::endl;
//                    exit(122);
//                }
//            }
        }

    }
}

std::vector<GateInfo> DeterministicPath::getGatesData()
{
    return m_gatesData;
}

void DeterministicPath::clearGateData()
{
    m_gatesData.clear();
}

void DeterministicPath::addGateData(GateInfo _gateInfo)
{
    m_gatesData.push_back(_gateInfo);
}

void DeterministicPath::setPath(std::vector<Gate *> _path)
{
    m_path = _path;
}

} // namespace LIMoSim
