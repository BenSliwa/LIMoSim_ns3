#ifndef SCENARIOS_H
#define SCENARIOS_H

#include <stdint.h>

namespace LIMoSim {
namespace Standalone {
namespace scenarios {

void aerialBaseStation(uint8_t _numCars = 5, uint8_t _run = 0);

void aerialBaseStation_mult(uint8_t _runs = 0);

void alignmentScenario_1();

void alignmentScenario_2();

void arriveScenario_1();

void arriveScenario_2();

void circleAroundScenario_1();

void circleAroundScenario_2();

void circleAroundScenario_3();

void circleAroundWithCollisionAvoidanceScenario_1();

void circleAroundWithCollisionAvoidanceScenario_2();

void collisionAvoidanceScenario_1();

void collisionAvoidanceScenario_2();

void collisionAvoidanceScenario_3();

void collisionAvoidanceScenario_4();

void cohesionScenario_1();

void cohesion2DRegionwise_1();

void demoCarScenario_1();

void finalPresentationScenario_1();

void fleeScenario_1 ();

void fleeAndSeekScenario_1();

void fleeWithCollisionAvoidanceScenario_1();

void flockingScenario_1();

void followCarScenario_1();

void hoverScenario_1();

void leaderFollowingScenario_1();

void leaderFollowingScenario_2();

void leaderFollowingScenario_3();

void leaderFollowingScenario_4();

void leaderFollowingWithCollisionAvoidanceScenario_1();

void mixedScenario_1();

void noOpScenario();

void randomWalkScenario_1();

void randomWalk_GaussMarkov_Scenario_1();

void randomWaypointScenario_1(double _h);

void randomWaypointScenario_2();

void raytracingTest();

void raytracingPredictionTest();

void seekScenario_1();

void seekScenario_2();

void seekWithSpeedScenario_1();

void seekWithSpeedScenario_2();

void separationScenario();

void wanderScenario();

void waypointRouteScenario_1();
void waypointRouteScenario_2();

} // namespace scenarios


namespace helpers {
using namespace LIMoSim::Standalone::scenarios;

    void configureScenario();
    void loadScenarioRegistry();
    void printAvailableScenarios();
} // namespace helpers

} // namespace standalone




namespace ScenarioHandlers {

void stop();
}

}

#endif // SCENARIOS_H
