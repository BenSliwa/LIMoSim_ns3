#include "randomwaypoint.h"
#include "LIMoSim/world/world.h"
#include "LIMoSim/mobility/routing/dijkstra.h"

namespace LIMoSim
{

RandomWaypoint::RandomWaypoint(Car *_car) :
    FollowPath(_car)
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void RandomWaypoint::initialize()
{
    std::cout << "RandomWaypoint::initialize" << std::endl;

    RoadPosition info = p_car->getRoadPosition();

    Gate *start = info.path.at(0);
    Gate *destination = 0;
    while(destination==0 || destination==start)
        destination = computeRandomDestination();

    Dijkstra dijkstra;
    std::vector<Routable*> graph = World::getInstance()->buildRoutingGraph();
    dijkstra.setGraph(graph);
    std::vector<Routable*> path = dijkstra.computeShortestPath(start, destination);
    for(unsigned int i=1; i<path.size(); i++)
    {
        Gate *gate = dynamic_cast<Gate*>(path.at(i));
        info.path.push_back(gate);
    }

    //
    p_car->setRoadPosition(info);
}

void RandomWaypoint::handleNodeReached(Node *_node)
{
    FollowPath::handleNodeReached(_node);
}

void RandomWaypoint::handleGateReached(Gate *_gate, Intersection *_intersection, LaneSegment *_lane)
{
    FollowPath::handleGateReached(_gate, _intersection, _lane);

    RoadPosition info = p_car->getRoadPosition();
    if(info.path.size()==1)
    {
        std::cout << "RandomWaypoint::handleGateReached no waypoints left" << std::endl;

        //
        Gate *start = info.path.at(0);
        Gate *destination = 0;
        while(destination==0 || destination==start)
            destination = computeRandomDestination();

        info.path.clear(); // the next gate is still required but it is also contained in the dijkstra path

        // compute a random waypoint from the next reachable gate
        Dijkstra dijkstra;
        std::vector<Routable*> graph = World::getInstance()->buildRoutingGraph();
        dijkstra.setGraph(graph);
        std::vector<Routable*> path = dijkstra.computeShortestPath(start, destination);
        for(unsigned int i=0; i<path.size(); i++)
        {
            Gate *gate = dynamic_cast<Gate*>(path.at(i));
            info.path.push_back(gate);
        }

        //
        p_car->setRoadPosition(info);

    }
}

/*************************************
 *           PRIVATE METHODS         *
 ************************************/


Gate* RandomWaypoint::computeRandomDestination()
{
    std::vector<Routable*> graph = World::getInstance()->buildRoutingGraph();
    int index = rand() * (long)(graph.size()) / RAND_MAX;

    Gate *gate = dynamic_cast<Gate*>(graph.at(index));

    return gate;
}


}
