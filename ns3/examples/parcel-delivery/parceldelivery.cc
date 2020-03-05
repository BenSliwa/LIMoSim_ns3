#include "parceldelivery.h"

#include <algorithm>
#include <random>

// ns3
#include "ns3/core-module.h"
#include <ns3/config-store.h>
#include <ns3/mobility-module.h>
#include <ns3/internet-module.h>
#include <ns3/yans-wifi-helper.h>

// LIMoSim ns3
#include "ns3/callbacks.h"
#include "ns3/ns3setuphelpers.h"
#include "ns3/limosimmobilitymodel.h"
#include "ns3/strategicmodels/ns3_truckdeliverystrategy.h"
#include "ns3/behaviors/ns3_behavior_delivery.h"
#include "ns3/applications/udptransciever.h"

// LIMoSim
#include "LIMoSim/simulation/performancemonitor.h"
#include "LIMoSim/utils/vector.h"
#include "LIMoSim/world/vehiclemanager.h"
#include "LIMoSim/world/world.h"
#include "LIMoSim/world/worldutils.h"
#include "ui/data/uidatamanager.h"

// Demo
#include "demo/deliverylistservice.h"
#include "demo/deliverysettingsservice.h"
#include "demo/behavior_delivery.h"
#include "demo/behavior_dock.h"
#include "demo/deliveryutils.h"


#include "standalone/qclidecorators.h"


#include <QCommandLineParser>

namespace LIMoSim {
namespace NS3 {
namespace Examples {
namespace ParcelDelivery {

inline std::string exampleName() {
    return "ParcelDelivery";
}

inline bool monitorPerformance() {
    return false;
}

NS_LOG_COMPONENT_DEFINE (exampleName());

void RxDrop(Ptr<const Packet> p) {
    std::cout << "packet dropped" << std::endl;
}

void setup(uint16_t _runCount, uint16_t _numOfUAVs, uint16_t _numOfDeliveries)
{
    using namespace delivery;
    using namespace Behaviors;


    LogComponentEnable(exampleName().c_str(), ns3::LOG_LEVEL_INFO);
    NS_LOG_INFO("setting up ns3 simulation setup of " << exampleName());
    QCommandLineParser parser;
    parser.setApplicationDescription("LIMoSim - ns3 - parcelDelivery");
    parser.addHelpOption();
    parser.addVersionOption();

    Standalone::qclidecorators::addGeneralOptions(parser);
    Standalone::qclidecorators::addSimulationOptions(parser);

    QCommandLineOption delivererCountOption(
                "deliverer-count",
                "Number of UAVs used for delivery",
                "number");
    QCommandLineOption deliveryCountOption(
                "delivery-count",
                "Number of random delivery targets",
                "number");
    QCommandLineOption usePredictionOption(
                "mobility-prediction",
                "Use trajectory prediction");
    QCommandLineOption deliveryListFileOption(
                "delivery-list-file",
                "File in which the delivery list is saved",
                "filepath");
    QCommandLineOption droneInteractionModeOption(
                "drone-interaction",
                "Drone interaction mode: ONSITE(0); ENROUTE(1)",
                "number");
    QCommandLineOption runCountOption(
                QStringList() << "r" << "run-count",
                "Run count",
                "number");

    parser.addOptions({
                          delivererCountOption,
                          deliveryCountOption,
                          usePredictionOption,
                          deliveryListFileOption,
                          droneInteractionModeOption,
                          runCountOption
                      });
    parser.parse(QCoreApplication::arguments());


    std::cout << "parsed" << std::endl;

    if (parser.isSet("help")) {
        std::cout << parser.helpText().toStdString() << std::endl;
        return;
    }

    if (!parser.errorText().isEmpty()) {
        std::cerr << parser.errorText().toStdString() << std::endl;
        return;
    }

    // Set sim params
    bool ok = true;
    uint16_t deliveryCount = parser.isSet(deliveryCountOption) ?
                parser.value(deliveryCountOption).toInt(&ok) :
                _numOfDeliveries;
    if (!ok) {
        std::cout<<"NS3::Examples::ParcelDelivery::setup: setting delivery count fallback value: 10"<<std::endl;
        deliveryCount = _numOfDeliveries;
        ok = true;
    }
    uint16_t delivererCount = parser.isSet(delivererCountOption) ?
                parser.value(delivererCountOption).toInt(&ok) :
                _numOfUAVs;
    if (!ok) {
        std::cout<<"NS3::Examples::ParcelDelivery::setup: setting deliverer count fallback value: 1"<<std::endl;
        delivererCount = 1;
        ok = true;
    }
    uint16_t droneInteractionMode = parser.isSet(droneInteractionModeOption) ?
                parser.value(droneInteractionModeOption).toInt(&ok) :
                0;
    if (!ok) {
        std::cout<<"NS3::Examples::ParcelDelivery::setup: setting drone interaction mode fallback value: ONSITE"<<std::endl;
        droneInteractionMode = 0;
        ok = true;
    }
    uint16_t runCount = parser.isSet(runCountOption) ?
                parser.value(runCountOption).toInt(&ok) :
                0;
    if (!ok) {
        std::cout<<"NS3::Examples::ParcelDelivery::setup: setting run count fallback value: 0"<<std::endl;
        runCount = _runCount;
        ok = true;
    }
    bool usePrediction = parser.isSet(usePredictionOption);



    std::vector<std::string> deliveryList = {};

    if (parser.isSet(deliveryListFileOption)) {
        FileHandler file;
        deliveryList = LIMoSim::delivery::utils::loadDeliveryListFromFile(parser.value(deliveryListFileOption).toStdString());
        std::cout << "Read " << deliveryList.size() << " delivery entries from delivery list file\n";
        deliveryCount = deliveryList.size();
    } else {

        auto buildings = World::getInstance()->getBuildings();
        size_t buildingSize = buildings.size();

        StringVector excludedBuildings = {
            "32853244",
            "32852781"
        };
        std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
        std::uniform_int_distribution<> dis(0, buildingSize);

        uint i = 0;
        Endpoints endpoints;

//        bool ok = false;
//        uint deliveryCount = parser.value(deliveryCountOption).toInt(&ok);
//        if (!ok) {
//            std::cout<<"Scenarios::Standalone::scenarios::parcelDeliveryScenario_1: setting delivery count fallback value: 10"<<std::endl;
//            deliveryCount = 10;
//        }
        RandomBuildingSelection buildingSelection = selectRandomBuildings(deliveryCount, excludedBuildings);
        endpoints = buildingSelection.buildingEndpoints;
        deliveryList = buildingSelection.buildingIds;
    }

    DeliverySettingsService* deliverySettings = DeliverySettingsService::getInstance();
    deliverySettings->setDeliverySetting(
                DeliverySettingsService::DeliverySettingKey::DRONE_INTERACTION_MODE,
                static_cast<DroneInteractionMode>(droneInteractionMode)
                );


    size_t deliveryListHash = hashDeliveryList(deliveryList);
    std::cout << "setting up ns3 simulation setup of " << exampleName() << " run count:" << runCount <<std::endl;
    Simulation::getInstance()->setName(exampleName() + "_" +
                                       "dh" + std::to_string(deliveryListHash) + "_" +
                                       "nu" + std::to_string(delivererCount) + "_"+
                                       "nd" + std::to_string(deliveryCount) + "_" +
                                       "pr" + std::to_string(usePrediction) + "_" +
                                       "di" + std::to_string(droneInteractionMode) + "_" +
                                       "r" + std::to_string(runCount))->setRunCount(runCount);

    // Print simulation parameters

    std::cout << "Scenario params: " <<std::endl;
    std::cout << "- num of UAVs: " << delivererCount << std::endl;
    std::cout << "- num of deliveries: " << deliveryCount << std::endl;
    std::cout << "- drone interaction: " << droneInteractionMode << std::endl;



    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults ();

    bool showProgress = true;

//    srand(static_cast<unsigned int>(time(nullptr)));
//    uint seed = static_cast<uint>(rand());
//    RngSeedManager::SetSeed (seed);
//    RngSeedManager::SetRun (static_cast<uint>(_runCount));

    // Network setup

    NS_LOG_INFO ("Creating nodes.");
    NodeContainer LIMoSimNodes;
    LIMoSimNodes.Create (delivererCount +1);

    // Simulation setup

    // Install mobility models on mobile nodes
    MobilityHelper mobilityHelper;
    mobilityHelper.SetMobilityModel("LIMoSim::NS3::LIMoSimMobility",
                                    "VehicleId", StringValue("T0"));
    mobilityHelper.Install(LIMoSimNodes.Get(0));
    for (uint uavIndex = 0; uavIndex < delivererCount; uavIndex++) {
        mobilityHelper.SetMobilityModel("LIMoSim::NS3::LIMoSimMobility",
                                        "VehicleId", StringValue("U" + std::to_string(uavIndex)));
        mobilityHelper.Install(LIMoSimNodes.Get(uavIndex + 1));
    }

    VehicleManager *vehicleManager = VehicleManager::getInstance();
    UI::Data::UIDataManager *uidm = UI::Data::UIDataManager::getInstance();
    using namespace delivery;

    vehicleManager->enableMobilityBroadcastHelper();

    mobility::car::DeliveryTruck *truck = vehicleManager->createDeliveryTruck(
                "T0",
                delivererCount,
                deliveryList
                );

    uidm->getVehicleData(truck->getId())->getShape()->setColor("red");
    // CAUTION: The ns3 truck delivery strategy is an ns3 object
    // wrapped in a smart pointer so deletion of the reference object
    // could have unforeseen consequences.
    Ptr<StrategicModels::NS3_TruckDeliveryStrategy> truckDeliveryStrategy = CreateObjectWithAttributes<StrategicModels::NS3_TruckDeliveryStrategy>(
                "TruckId", StringValue("T0")
                );

    truckDeliveryStrategy->initializeMobility("T0");
    LIMoSimNodes.Get(0)->AddApplication(truckDeliveryStrategy);
    truckDeliveryStrategy->SetStartTime(Seconds(0));
    truck->setStrategy(&*truckDeliveryStrategy);
    truck->initialize();


    std::vector<NS3_Behavior_Delivery*> uavDeliveryBehaviors;
    for (uint i = 0 ; i < delivererCount; i++) {
        UAV *uav = vehicleManager->createUAV("U"+ std::to_string(i));
        uav->setPosition(truck->getPosition() + Vector3d(0,i*7,10));

        Ptr<Behaviors::NS3_Behavior_Delivery> behavior = CreateObjectWithAttributes<Behaviors::NS3_Behavior_Delivery>(
                    "TruckId", StringValue("T0"),
                    "VehicleId", StringValue(uav->getId())
                    );

        behavior->Setup("T0", usePrediction, uav->getId());
        uav->setBehavior(&*behavior);
        behavior->setTruckDeliveryStrategy(&*truckDeliveryStrategy);
        behavior->setCommunicationsEnabled(false);
        uavDeliveryBehaviors.push_back(&*behavior);
        LIMoSimNodes.Get(i+1)->AddApplication(behavior);
        behavior->SetStartTime(Seconds(0));
    }
    truckDeliveryStrategy->setUavDeliveryBehaviors(uavDeliveryBehaviors);
    truckDeliveryStrategy->setCommunicationsEnabled(false);

    // Simulation setup end

    if (monitorPerformance()) {
        PerformanceMonitor::getInstance()->reset();
        PerformanceMonitor::getInstance()->setupWithExport("../results/Performance_" +
                                                           Simulation::getInstance()->getName() + "_" +
                                                           std::to_string(_runCount) +
                                                           ".csv");
    }

    // Display simulator time progress
    if (showProgress)
    {
        uint32_t status_period = 1;
        Simulator::Schedule(Seconds(1),&Callbacks::PrintProgress, status_period);
    }

    setupIpv4TrafficAnimation();
    startSimulation();

}

} // namespace ParcelDelivery
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim
