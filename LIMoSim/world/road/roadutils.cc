#include "roadutils.h"

#include <random>

#include "LIMoSim/world/world.h"



namespace LIMoSim {
namespace RoadUtils {

static bool initialized = false;
static uint roadCount = 0;
static std::mt19937 rgen;
static std::uniform_int_distribution<int> idist;
static std::vector<Routable*> graph;

void init()
{
    if (!initialized) {
        resetLane();
        initialized = true;
        std::random_device rdev;
        rgen = std::mt19937 (rdev());
        idist = std::uniform_int_distribution<int> (0, static_cast<int>(roadCount));
    }
}

void resetLane()
{
    World *world = World::getInstance();
    graph = world->buildRoutingGraph();
}

Gate* randomValidGate() {
    if (!initialized) {
        init();
    }
    RoadPosition roadPosition;
    int index = (rand() * (long)(graph.size()) / RAND_MAX);
    //   index = idist(rgen);
    Gate *gate = dynamic_cast<Gate*>(graph.at(index));
    return gate;
}


RoadPosition randomValidRoadPosition()
{
    if (!initialized) {
        init();
    }

   RoadPosition roadPosition;
   int index = (rand() * (long)(graph.size()) / RAND_MAX);
   Gate *gate = dynamic_cast<Gate*>(graph.at(index));
   roadPosition.path.push_back(gate); // no path for RandomDirection
   roadPosition.laneSegment = gate->getEndpoint()->getOwner()->getLanes(LANE_TYPE::FORWARD).at(0);
   roadPosition.offset_m = 0;
   return roadPosition;
}

RoadPosition sameRoadPosition()
{
    static int genCount = 1;
    static RoadPosition roadPosition = randomValidRoadPosition();
    roadPosition.offset_m = 5 * genCount++;
    return roadPosition;
}



} // namespace RoadUtils
} // namespace LIMoSim
