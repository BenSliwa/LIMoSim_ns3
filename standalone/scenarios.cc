#include "scenarios.h"

#include "standalone/scenarioregistry.h"
#include "standalone/qclidecorators.h"

#include "LIMoSim/mobility/uav/reynolds/behaviors.h"
#include "LIMoSim/world/world.h"
#include "LIMoSim/world/worldutils.h"
#include "LIMoSim/world/road/road.h"
#include "LIMoSim/world/road/lanesegment.h"
#include "LIMoSim/world/road/roadsegment.h"
#include "LIMoSim/world/vehiclemanager.h"
#include "LIMoSim/world/road/roadutils.h"
#include "LIMoSim/world/raytracing/raytracingprediction.h"
#include "LIMoSim/mobility/routing/tsp.h"
#include "ui/data/uidatamanager.h"
#include "ui/data/defaultshapes.h"
#include "LIMoSim/simulation/performancemonitor.h"
#include "LIMoSim/utils/vector.h"

#include "aerialbasestation.h"
#include "raytracingtest.h"
#include "raytracingpredictiontest.h"

#include <algorithm>

#include <QCommandLineParser>
#include <QString>

namespace LIMoSim {
namespace Standalone {


namespace helpers {


void configureScenario() {
    QCommandLineParser parser;
    parser.setApplicationDescription("LIMoSim standalone");
    parser.addHelpOption();
    parser.addVersionOption();

    helpers::loadScenarioRegistry();

    qclidecorators::addGeneralOptions(parser);
    qclidecorators::addSimulationOptions(parser);

    parser.parse(QCoreApplication::arguments());
    if (!parser.isSet(cliOptions::scenario())) {
        parser.process(QCoreApplication::arguments());
    }

    if (parser.isSet(cliOptions::listScenarios())) {
        helpers::printAvailableScenarios();
        exit(0);
    }

    if (parser.isSet(cliOptions::scenario())) {
        Scenario s = ScenarioRegistry::getInstance()->getScenario(parser.value(cliOptions::scenario()).toStdString());
        if (s) {
            s();
        } else {
            std::cerr << "scenario " << parser.value(cliOptions::scenario()).toStdString() << " was not found!" << std::endl;
        }
    }



//    noOpScenario();

//    aerialBaseStation_mult();
//    alignmentScenario_1();
//    alignmentScenario_2();

//    arriveScenario_1();
//    arriveScenario_2();

//    circleAroundScenario_1();
//    circleAroundScenario_2();
//    circleAroundScenario_3();
//    circleAroundWithCollisionAvoidanceScenario_1();
//    circleAroundWithCollisionAvoidanceScenario_2();

//    cohesionScenario_1();

//    cohesion2DRegionwise_1();

//    collisionAvoidanceScenario_1();
//    collisionAvoidanceScenario_2();
//    collisionAvoidanceScenario_3();
//    collisionAvoidanceScenario_4();

//    demoCarScenario_1();

//    finalPresentationScenario_1();
//    fleeScenario_1();

//    fleeAndSeekScenario_1();

//    fleeWithCollisionAvoidanceScenario_1();

//    flockingScenario_1();

//    followCarScenario_1();
//    hoverScenario_1();

//    leaderFollowingScenario_1();
//    leaderFollowingScenario_2();
//    leaderFollowingScenario_3();
//    leaderFollowingScenario_4();

//    leaderFollowingWithCollisionAvoidanceScenario_1();

//    parcelDeliveryScenario_1();
//    parcelDeliveryScenario_2();
//    parcelDeliveryScenario_withoutPrediction();

//    randomWalkScenario_1();

//    randomWaypointScenario_1();

//    randomWaypointScenario_2();

//    randomWalk_GaussMarkov_Scenario_1();

//    raytracingTest();

//    raytracingPredictionTest();

//    seekScenario_1();
//    seekScenario_2();

//    seekWithSpeedScenario_1();
//    seekWithSpeedScenario_2();

//    separationScenario();

//    wanderScenario();

//    waypointRouteScenario_1();
//    waypointRouteScenario_2();
}

void loadScenarioRegistry()
{
    ScenarioRegistry *registry = ScenarioRegistry::getInstance();

    registry->registerScenario("circleAround", circleAroundScenario_1);
    registry->registerScenario("noop", noOpScenario);
    registry->registerScenario("followCar", followCarScenario_1);
}

void printAvailableScenarios()
{
    std::cout << "List of available standalone scenarios:\n";
    ScenarioRegistry::getInstance()->printScenarios();
}

} // namespace registryHelpers



namespace scenarios {

using namespace UI::Data;

void aerialBaseStation(uint8_t _numCars, uint8_t _run)
{
    std::string scenarioName = "AerialBaseStation";

    std::cout << "starting scenario " << scenarioName
              << " " << std::to_string(_numCars) << " cars"
              << " run "
              << std::to_string(_run) << std::endl;

    Simulation::getInstance()->setName(scenarioName +
                                       std::to_string(_numCars) + "Cars")
            ->setRunCount(_run);

    VehicleManager *vehicleManager = VehicleManager::getInstance();
    UIDataManager *uidm = UIDataManager::getInstance();

    PerformanceMonitor::getInstance()->reset();
    PerformanceMonitor::getInstance()->setupWithExport("../results/Performance_" +
                                                       scenarioName +
                                                       std::to_string(_numCars) + "Cars_" +
                                                       std::to_string(_run) +
                                                       ".csv");

//    RoadUtils::resetLane();
    RoadPosition roadPosition;
    for (uint8_t carIdx = 0; carIdx < _numCars; carIdx++) {
        Car *car = vehicleManager->createCar("C" + std::to_string(carIdx));
        uidm->getVehicleData(car->getId())->getShape()->setColor("red");
        roadPosition = RoadUtils::randomValidRoadPosition();
        car->setRoadPosition(roadPosition);
        car->initialize();
    }

    UAV *uav = vehicleManager->createUAV("U0");
    uidm->getVehicleData(uav->getId())->getShape()->setColor("yellow");
    uav->setPosition(Vector3d(100, 100, 50));
    uav->setBehavior(new Behavior_Cohesion2D(1500));

}



void aerialBaseStation_mult(uint8_t _runs)
{

    auto scenario = new Standalone::AerialBaseStation ();
    scenario->setup(5);
}


void alignmentScenario_1()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U0");
    UAV *uav1= vehicleManager->createUAV("U1");

    uav0->setPosition(Vector3d(100, 100, 50));
    uav1->setPosition(Vector3d(80, 80, 50));

    uav0->setBehavior(new Behavior_Wander());
    uav1->setBehavior(new Behavior_Alignment());

}

void alignmentScenario_2()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U2");
    UAV *uav1= vehicleManager->createUAV("U3");

    uav0->setPosition(Vector3d(100, 100, 50));
    uav1->setPosition(Vector3d(80, 80, 50));

    uav0->setBehavior(new Behavior_RandomWalk());
    uav1->setBehavior(new Behavior_Alignment());

}

void arriveScenario_1()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U0");


    uav0->setPosition(Vector3d(300, 350, 50));
    uav0->setBehavior(new Behavior_Arrive(Vector3d(0,0,50),1));
}

void arriveScenario_2()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();
    Simulation::getInstance()->setName("Arrive2");

    UAV *uav0 = vehicleManager->createUAV("U0");


    uav0->setPosition(Vector3d(0, 0, 0));
    uav0->setBehavior(makeSelfAligned(new Behavior_Arrive(Vector3d(40, 50, 60), 50)));
}

void circleAroundScenario_1()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();
    Simulation::getInstance()->setName("CircleAround2");

    UAV *uav0 = vehicleManager->createUAV("U0");

//    uav0->setPosition(Vector3d(100, 100, 250));
    uav0->setPosition(Vector3d(0, -70, 50));
    uav0->setBehavior(new Behavior_CircleAround(Vector3d(0,0,50), 70));
}

void circleAroundScenario_2()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U1");
    UAV *uav1= vehicleManager->createUAV("U2");

    uav0->setPosition(Vector3d(200,200, 50));
    uav1->setPosition(Vector3d(200,150, 50));

    uav0->setAccelerationMax(5);
    uav0->setVelocityMax(5);

    uav0->setBehavior(new Behavior_CircleAround(Vector3d(0,0,50), 50));
    uav1->setBehavior(new Behavior_CircleAround(Target(uav0->getId()), 50));
}

void circleAroundScenario_3()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U1");
    UAV *uav1= vehicleManager->createUAV("U2");

    uav1->getModel()->getLocomotion()->setAccelerationComponentsCoeffMax(
                Vector3d(6,2,2)
                );
    uav1->getModel()->getLocomotion()->setVelocityComponentsCoeffMax(
                Vector3d(6,2,2)
                );

    uav0->setPosition(Vector3d(200,200, 50));
    uav1->setPosition(Vector3d(200,150, 50));

//    uav0->setAccelerationMax(5);
//    uav0->setVelocityMax(5);

    uav0->setBehavior(new Behavior_CircleAround(Vector3d(0,0,50), 50));
    uav1->setBehavior(new Behavior_CircleAround(Vector3d(0,0,50), 50));
}

void circleAroundWithCollisionAvoidanceScenario_1() {

    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U3");
    UAV *uav1= vehicleManager->createUAV("U4");

    uav0->setPosition(Vector3d(0,50, 50));
    uav1->setPosition(Vector3d(200,50, 50));

    uav1->setBehavior(Behavior_CollisionAvoidance::composeWith(new Behavior_CircleAround(Vector3d(0,0,50), 50)));
}

void circleAroundWithCollisionAvoidanceScenario_2() {

    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U5");
    UAV *uav1= vehicleManager->createUAV("U6");

    uav0->setPosition(Vector3d(200,0, 50));
    uav1->setPosition(Vector3d(0,200, 50));

    uav0->setAccelerationMax(5);
    uav0->setVelocityMax(5);

    uav0->setBehavior(new Behavior_CircleAround(Vector3d(0,0,50), 51));
    //    uav0->setBehavior(Behavior_CollisionAvoidance::composeWith(new Behavior_CircleAround(Vector3d(0,0,50), 50)));
    uav1->setBehavior(Behavior_CollisionAvoidance::composeWith(new Behavior_CircleAround(Vector3d(0,0,50), 50)));
}

void cohesionScenario_1()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U0");
    UAV *uav1= vehicleManager->createUAV("U1");

    UAV *uav2 = vehicleManager->createUAV("U2");
    UAV *uav3 = vehicleManager->createUAV("U3");

    uav0->setPosition(Vector3d(250, 50, 50));
    uav1->setPosition(Vector3d(300, 0, 50));
    uav2->setPosition(Vector3d(350, 0, 50));
    uav3->setPosition(Vector3d(400, 32, 50));

    uav0->setBehavior(new Behavior_Cohesion());
    uav1->setBehavior(new Behavior_Cohesion());
    uav2->setBehavior(new Behavior_Cohesion());
    uav3->setBehavior(new Behavior_Cohesion());
}

void cohesion2DRegionwise_1()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();
    UIDataManager *uidm = UIDataManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U0");
    UAV *uav1= vehicleManager->createUAV("U1");

    uidm->getVehicleData(uav0->getId())->getShape()->setColor("yellow");
    uidm->getVehicleData(uav1->getId())->getShape()->setColor("yellow");

    uint _numCars = 10;

    RoadPosition roadPosition;
    for (uint8_t carIdx = 0; carIdx < _numCars; carIdx++) {
        Car *car = vehicleManager->createCar("C" + std::to_string(carIdx));
        uidm->getVehicleData(car->getId())->getShape()->setColor("red");
        roadPosition = RoadUtils::randomValidRoadPosition();
        car->setRoadPosition(roadPosition);
        car->initialize();
    }

    uav0->setBehavior(new Behavior_Cohesion2DRegionwise(Vector3d(-100, 3000), 8000, 500));
    uav1->setBehavior(new Behavior_Cohesion2DRegionwise(Vector3d(501, 3000), 8000, 900));



    uav0->setPosition(Vector3d(200, 300, 50));
    uav1->setPosition(Vector3d(700, 300, 50));
}

void collisionAvoidanceScenario_1()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U0");
    UAV *uav1= vehicleManager->createUAV("U1");


    uav0->setPosition(Vector3d(0, 100, 50));
    uav1->setPosition(Vector3d(100,0, 50));

    uav0->setBehavior(Behavior_CollisionAvoidance::composeWith(new Behavior_Seek(Vector3d(200,100,50))));
    uav1->setBehavior(Behavior_CollisionAvoidance::composeWith(new Behavior_Seek(Vector3d(100,200,50))));
}

void collisionAvoidanceScenario_2()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U2");
    UAV *uav1= vehicleManager->createUAV("U3");


    uav0->setPosition(Vector3d(0, 0, 50));
    uav1->setPosition(Vector3d(200,0, 50));

    uav0->setBehavior(Behavior_CollisionAvoidance::composeWith(new Behavior_Seek(Vector3d(200,0,50))));
    uav1->setBehavior(Behavior_CollisionAvoidance::composeWith(new Behavior_Seek(Vector3d(0,0,50))));
}

void collisionAvoidanceScenario_3()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U4");
    UAV *uav1= vehicleManager->createUAV("U5");


    uav0->setPosition(Vector3d(250, 0, 50));
    uav1->setPosition(Vector3d(350, 0, 50));

    uav1->setAccelerationMax(5);
    uav1->setVelocityMax(5);

    uav0->setBehavior(Behavior_CollisionAvoidance::composeWith(new Behavior_Seek(Vector3d(500,0,50))));
    uav1->setBehavior(Behavior_CollisionAvoidance::composeWith(new Behavior_Seek(Vector3d(500,0,50))));
}

void collisionAvoidanceScenario_4()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U6");
    UAV *uav1= vehicleManager->createUAV("U7");


    uav0->setPosition(Vector3d(300,100, 50));
    uav1->setPosition(Vector3d(350,100, 50));

    uav0->setBehavior(Behavior_CollisionAvoidance::composeWith(new Behavior_Arrive(Vector3d(400,100,50))));
}

void demoCarScenario_1()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();
    Simulation::getInstance()->setName("DemoCar Scenario 1");
    vehicleManager->setDeterministicCarPathEnabled(true);

    Car *car = vehicleManager->createCar("C0");

    RoadPosition roadPosition;
    roadPosition = RoadUtils::randomValidRoadPosition();
    car->setRoadPosition(roadPosition);
    car->initialize();
}

void fleeScenario_1()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U0");
    UAV *uav1= vehicleManager->createUAV("U1");


    uav0->setPosition(Vector3d(250, 250, 50));
    uav1->setPosition(Vector3d(280, 280, 50));

    uav0->setBehavior(new Behavior_Flee(Target(uav1->getId())));
}

void fleeAndSeekScenario_1()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U1");
    UAV *uav1= vehicleManager->createUAV("U2");
    UAV *uav2= vehicleManager->createUAV("U3");

    UIDataManager* uiData = UIDataManager::getInstance();

    //uiData->registerVehicles();

    uiData->getVehicleData("U1")->getShape()->setColor("red");
    uiData->getVehicleData("U2")->getShape()->setColor("green");
    uiData->getVehicleData("U3")->getShape()->setColor("blue");

    uav1->getModel()->getLocomotion()->setAccelerationComponentsCoeffMax(
                Vector3d(6,2,2)
                );
    uav1->getModel()->getLocomotion()->setVelocityComponentsCoeffMax(
                Vector3d(6,2,2)
                );
    uav1->getModel()->getLocomotion()->setBrakeAccelerationMax(Vector3d(2,2,5));


    uav0->setPosition(Vector3d(250, 250, 50));
    uav1->setPosition(Vector3d(280, 280, 50));
    uav2->setPosition(Vector3d(280, 250, 50));

//    uav1->setBehavior(new Behavior_Flee(Target(uav0->getId())));
    uav0->setBehavior(new Behavior_RandomWalk());
    uav1->setBehavior(makeSelfAligned(new Behavior_Seek(Target(uav0->getId()))));
    uav2->setBehavior(makeSelfAligned(new Behavior_Seek(Target(uav0->getId()))));

}

void fleeWithCollisionAvoidanceScenario_1()
{

    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U0");
    UAV *uav1= vehicleManager->createUAV("U1");
    UAV *uav2 = vehicleManager->createUAV("U2");

    uav0->setPosition(Vector3d(300, 200, 50));
    uav1->setPosition(Vector3d(200, 200, 50));
    uav2->setPosition(Vector3d(100, 200, 50));

    uav1->setBehavior(Behavior_CollisionAvoidance::composeWith(new Behavior_Flee(Target(uav0->getId()))));
}

void flockingScenario_1()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U0");
    UAV *uav1= vehicleManager->createUAV("U1");

    UAV *uav2 = vehicleManager->createUAV("U2");
    UAV *uav3 = vehicleManager->createUAV("U3");

    uav0->setPosition(Vector3d(250, 50, 50));
    uav1->setPosition(Vector3d(300, 0, 50));
    uav2->setPosition(Vector3d(350, 0, 50));
    uav3->setPosition(Vector3d(400, 32, 50));

    uav0->setBehavior(new Behavior_Flocking());
    uav1->setBehavior(new Behavior_Flocking());
    uav2->setBehavior(new Behavior_Flocking());
    uav3->setBehavior(new Behavior_Flocking());
}

void followCarScenario_1()
{
    // Define command line parameters specific to this scenario
    QCommandLineParser parser;
    parser.addVersionOption();
    qclidecorators::addGeneralOptions(parser);
    qclidecorators::addSimulationOptions(parser);
    QCommandLineOption pairCountOption(
                QStringList() << "u" << "pair-count",
                "Number of random delivery targets",
                "number");
    QCommandLineOption usePredictionOption(
                "mobility-prediction",
                "Use trajectory prediction");

    // Scenario specific cli parameters handling
    parser.addOptions({pairCountOption, usePredictionOption});
    parser.process(QCoreApplication::arguments());
    bool ok = false;
    uint pairCount = parser.isSet(pairCountOption) ? parser.value(pairCountOption).toUInt(&ok) : 0;
    if (!ok) {
        std::cout<<"Scenarios::Standalone::scenarios::followCarScenario_1: setting pair count fallback value: 1"<<std::endl;
        pairCount = 1;
    }
    bool usePrediction = parser.isSet(usePredictionOption);


    // Scenario start
    Simulation::getInstance()->setName(std::string("FollowCar") +
                                       "nu" + std::to_string(pairCount) + "_"+
                                       "pr" + std::to_string(usePrediction))->setRunCount(0);
    VehicleManager *vehicleManager = VehicleManager::getInstance();
    vehicleManager->enableMobilityBroadcastHelper();
//    UIDataManager *uidm = UIDataManager::getInstance();

    for (uint i = 0; i < pairCount; i++) {
        Car *car = vehicleManager->createCar("T"+std::to_string(i));
        UAV *uav = vehicleManager->createUAV("U"+std::to_string(i));
        RoadPosition roadPosition = RoadUtils::randomValidRoadPosition();
        car->setRoadPosition(roadPosition);
        car->initialize();
        uav->setBehavior(new Behavior_FollowAtElevation(car->getId()));
    }

    int vehicles = 30;
    for(int i=0; i<vehicles; i++)
    {
        Car *car = vehicleManager->createCar("C" + QString::number(i).toStdString());
        RoadPosition roadPosition = RoadUtils::randomValidRoadPosition();
        car->setRoadPosition(roadPosition);
        car->initialize();


        int offset = -10 + rand() * (long)(20) / RAND_MAX;
        double maxSpeed = 30+offset;
        //std::cout << car->getId() << "\t" << maxSpeed << std::endl;
        car->setMaxSpeed((maxSpeed)/3.6);
    }
}

void hoverScenario_1() {
    VehicleManager *vehicleManager = VehicleManager::getInstance();
    Simulation::getInstance()->setName("Hover1");

    UAV *uav0 = vehicleManager->createUAV("U0");


    uav0->setPosition(Vector3d(0, 0, 0));
    uav0->setBehavior((new Behavior_Arrive(Vector3d(50,0,20), 30)));
}

void leaderFollowingScenario_1()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U0");
    UAV *uav1= vehicleManager->createUAV("U1");

    uav0->setPosition(Vector3d(100, 100, 50));
    uav1->setPosition(Vector3d(200, 100, 50));

    uav0->setBehavior(new Behavior_Seek(Vector3d(400,100,50)));
    uav1->setBehavior(new Behavior_LeaderFollowing(uav0->getId()));
}

void leaderFollowingScenario_2()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U2");
    UAV *uav1= vehicleManager->createUAV("U3");
    UAV *uav2 = vehicleManager->createUAV("U4");
    UAV *uav3= vehicleManager->createUAV("U5");

    uav0->setPosition(Vector3d(100, 100, 50));
    uav1->setPosition(Vector3d(200, 110, 50));
    uav2->setPosition(Vector3d(200, 90, 50));
    uav3->setPosition(Vector3d(0, 200, 100));

    uav0->setBehavior(new Behavior_Seek(Vector3d(400,100,50)));
    uav1->setBehavior(new Behavior_LeaderFollowing(uav0->getId()));
    uav2->setBehavior(new Behavior_LeaderFollowing(uav0->getId()));
    uav3->setBehavior(new Behavior_LeaderFollowing(uav0->getId()));
}

void leaderFollowingScenario_3()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U6");
    UAV *uav1= vehicleManager->createUAV("U7");
    UAV *uav2 = vehicleManager->createUAV("U8");
    UAV *uav3= vehicleManager->createUAV("U9");

    uav0->setPosition(Vector3d(100, 100, 50));
    uav1->setPosition(Vector3d(200, 110, 50));
    uav2->setPosition(Vector3d(200, 90, 50));
    uav3->setPosition(Vector3d(0, 200, 100));

    uav0->setBehavior(new Behavior_RandomWalk());
    uav1->setBehavior(new Behavior_LeaderFollowing(uav0->getId()));
    uav2->setBehavior(new Behavior_LeaderFollowing(uav0->getId()));
    uav3->setBehavior(new Behavior_LeaderFollowing(uav0->getId()));
}

void leaderFollowingScenario_4()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U10");
    UAV *uav1= vehicleManager->createUAV("U11");
    UAV *uav2 = vehicleManager->createUAV("U12");
    UAV *uav3= vehicleManager->createUAV("U13");

    uav0->setPosition(Vector3d(300, 100, 50));
    uav1->setPosition(Vector3d(300, 110, 50));
    uav2->setPosition(Vector3d(300, 90, 50));
    uav3->setPosition(Vector3d(200, 200, 50));

    //    uav0->setBehavior(new Behavior_RandomWalk());
    uav0->setBehavior(new Behavior_LeaderFollowing(uav3->getId()));
    uav1->setBehavior(new Behavior_LeaderFollowing(uav0->getId()));
    uav2->setBehavior(new Behavior_LeaderFollowing(uav1->getId()));
    uav3->setBehavior(new Behavior_LeaderFollowing(uav2->getId()));
}

void leaderFollowingWithCollisionAvoidanceScenario_1()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U0");
    UAV *uav1= vehicleManager->createUAV("U1");
    UAV *uav2 = vehicleManager->createUAV("U2");
    UAV *uav3= vehicleManager->createUAV("U3");

    uav0->setPosition(Vector3d(100, 100, 50));
    uav1->setPosition(Vector3d(200, 110, 50));
    uav2->setPosition(Vector3d(200, 90, 50));
    uav3->setPosition(Vector3d(0, 200, 100));

    uav0->setBehavior(new Behavior_Seek(Vector3d(400,100,50)));
    uav1->setBehavior(Behavior_CollisionAvoidance::composeWith(new Behavior_LeaderFollowing(uav0->getId())));
    uav2->setBehavior(Behavior_CollisionAvoidance::composeWith(new Behavior_LeaderFollowing(uav0->getId())));
    uav3->setBehavior(Behavior_CollisionAvoidance::composeWith(new Behavior_LeaderFollowing(uav0->getId())));
}

void mixedScenario_1()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U0");
    UAV *uav1= vehicleManager->createUAV("U1");

    UAV *uav2 = vehicleManager->createUAV("U2");
    UAV *uav3 = vehicleManager->createUAV("U3");

    uav0->setPosition(Vector3d(250, 50, 50));
    uav1->setPosition(Vector3d(300, 0, 50));
    uav2->setPosition(Vector3d(350, 0, 50));
    uav3->setPosition(Vector3d(400, 32, 50));

    uav0->setBehavior(new Behavior_Flocking());
    uav1->setBehavior(new Behavior_Separation());
    uav2->setBehavior(new Behavior_CircleAround(Vector3d(0,0,50), 50));
    uav3->setBehavior(new Behavior_Flocking());
}


void noOpScenario()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U0");
    UAV *uav1= vehicleManager->createUAV("U1");

    uav0->setPosition(Vector3d(100, 0, 50));
    uav1->setPosition(Vector3d(0, 100, 50));

    uav0->setBehavior(new Behavior_NoOp());
    uav1->setBehavior(new Behavior_NoOp());

}


void randomWalkScenario_1()
{
    using namespace raytracing;

    Simulation::getInstance()->setName("RandomWalk")->setRunCount(0);
    VehicleManager *vehicleManager = VehicleManager::getInstance();
    World *world = World::getInstance();

    for (uint i = 0; i < 10; i++) {
        UAV *uav = vehicleManager->createUAV("U"+std::to_string(i));
        uav->setPosition(Vector3d(300 + 10 *i, 200 + 50 * (i%5), 20));
        uav->setBehavior(new Behavior_RandomWalk(Vector3d(world->getBoxMin().x,
                                                           world->getBoxMin().y,
                                                           10),
                                                  Vector3d(world->getBoxMax().x,
                                                           world->getBoxMax().y,
                                                           30)));
        RaytracingPrediction::getInstance()->startMonitoring(uav->getId());
    }

//    UAV *uav0 = vehicleManager->createUAV("U0");
//    uav0->setPosition(Vector3d(250, 250, 50));

//    UAV *uav1 = vehicleManager->createUAV("U1");
//    uav1->setPosition(Vector3d(150, 150, 10));

//    uav0->setBehavior(new Behavior_RandomWalk(Vector3d(world->getBoxMin().x,
//                                                       world->getBoxMin().y,
//                                                       10),
//                                              Vector3d(world->getBoxMax().x,
//                                                       world->getBoxMax().y,
//                                                       30)));
//    uav1->setBehavior(new Behavior_RandomWalk(Vector3d(world->getBoxMin().x,
//                                                       world->getBoxMin().y,
//                                                       10),
//                                              Vector3d(world->getBoxMax().x,
//                                                       world->getBoxMax().y,
//                                                       30)));
//    RaytracingPrediction::getInstance()->startMonitoring("U0");
//    RaytracingPrediction::getInstance()->startMonitoring("U1");
}

void randomWalk_GaussMarkov_Scenario_1()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U0");
    uav0->setPosition(Vector3d(250, 250, 50));

    uav0->setBehavior(new Behavior_GaussMarkovRandomWalk());
}

void randomWaypointScenario_1(double _h)
{
    using namespace raytracing;

    VehicleManager *vehicleManager = VehicleManager::getInstance();
    UIDataManager *uidm = UIDataManager::getInstance();

    World *world = World::getInstance();

    double h = _h;

    for (uint i = 0; i < 10; i++) {
        UAV *uav = vehicleManager->createUAV("U"+std::to_string(i));
        uav->setPosition(Vector3d(300 + 10 *i, 200 + 50 * (i%5), h));
        uav->setBehavior(new Behavior_RandomWaypoint(Vector3d(world->getBoxMin().x,
                                                           10,
                                                           h),
                                                  Vector3d(world->getBoxMax().x,
                                                           world->getBoxMax().y,
                                                           h)));

        uidm->getVehicleData(uav->getId())->getShape()->setColor("red");
        RaytracingPrediction::getInstance()->startMonitoring(uav->getId());
    }
}

void randomWaypointScenario_2() {
    VehicleManager *vehicleManager = VehicleManager::getInstance();
    Simulation::getInstance()->setName("RandomWaypoint2");
    World *world = World::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U0");


    uav0->setPosition(Vector3d(0, 0, 0));
    uav0->setBehavior(new Behavior_RandomWaypoint(Vector3d(0, 0, 5),
                                              Vector3d(100, 100, 30)));
}

void raytracingTest()
{
    auto scenario = new Standalone::RaytracingTest();
    scenario->setup(0,10,50,175);
}

void raytracingPredictionTest()
{
    auto scenario = new Standalone::RaytracingPredictionTest();
    scenario->setup();
}

void seekScenario_1()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U0");
    uav0->setPosition(Vector3d(300, 250, 50));

    //    uav0->setBehavior(new Behavior_Seek(Vector3d(0,0,50)));
    uav0->setBehavior(makeSelfAligned(new Behavior_Seek(Vector3d(0,0,50))));
}

void seekScenario_2()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U1");
    UAV *uav1 = vehicleManager->createUAV("U2");
    uav0->setPosition(Vector3d(350, 250, 50));
    uav1->setPosition(Vector3d(100, 50, 50));

    uav0->setBehavior(new Behavior_Seek(Target(uav1->getId())));
}

void seekWithSpeedScenario_1()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U0");
    UAV *uav1 = vehicleManager->createUAV("U1");
    uav0->setPosition(Vector3d(350, 250, 50));
    uav1->setPosition(Vector3d(350, 200, 50));

    uav0->setBehavior(new Behavior_SeekWithSpeed(Vector3d(0,0,50), 15));
    uav1->setBehavior(new Behavior_SeekWithSpeed(Vector3d(0,0,50), 10));
}

void seekWithSpeedScenario_2()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();
    Simulation::getInstance()->setName("SeekWithSpeed2");

    UAV *uav0 = vehicleManager->createUAV("U0");
    uav0->setPosition(Vector3d(0, 0, 0));

    uav0->setBehavior(new Behavior_SeekWithSpeed(Vector3d(0,0,6), 1.2));
}

void separationScenario()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U0");
    UAV *uav1= vehicleManager->createUAV("U1");

    UAV *uav2 = vehicleManager->createUAV("U2");
    UAV *uav3 = vehicleManager->createUAV("U3");

    uav0->setPosition(Vector3d(200, 50, 50));
    uav1->setPosition(Vector3d(200, 90, 50));
    uav2->setPosition(Vector3d(240, 50, 50));
    uav3->setPosition(Vector3d(240, 90, 50));

    uav0->setBehavior(new Behavior_Separation());
    uav1->setBehavior(new Behavior_Separation());
    uav2->setBehavior(new Behavior_Separation());
    uav3->setBehavior(new Behavior_Separation());

}

void wanderScenario()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();

    UAV *uav0 = vehicleManager->createUAV("U0");

    uav0->setPosition(Vector3d(200, 200, 50));

    uav0->setBehavior(new Behavior_Wander());
}



void waypointRouteScenario_1()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();
    Simulation::getInstance()->setName("WaypointRoute1");

    UAV *uav0 = vehicleManager->createUAV("U0");

    uav0->setPosition(Vector3d(0, 0, 0));
    uav0->getModel()->getLocomotion()->setVelocityMax(6.0);

    Waypoints waypoints {Vector3d(0,0,50), Vector3d(0,0,5), Vector3d(0,0,5)};
    uav0->setBehavior((new Behavior_WaypointRoute(waypoints, 20, 1e-1)));
}

void waypointRouteScenario_2()
{
    VehicleManager *vehicleManager = VehicleManager::getInstance();
    Simulation::getInstance()->setName("WaypointRoute2");

    UAV *uav0 = vehicleManager->createUAV("U0");

    uav0->setPosition(Vector3d(0, 0, 0));
    uav0->getModel()->getLocomotion()->setAccelerationMax(8);
    uav0->getModel()->getLocomotion()->setVelocityMax(10.0);

    Waypoints waypoints {Vector3d(0,0,4), Vector3d(0,0,5), Vector3d(0,0,6)};
    uav0->setBehavior((new Behavior_WaypointRoute(waypoints, 20, 0.5)));
}

}
}

namespace ScenarioHandlers {

void stop()
{
    Simulation::getInstance()->stop();
}

}

}
