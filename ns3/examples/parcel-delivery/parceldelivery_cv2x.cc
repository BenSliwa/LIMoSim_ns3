#include "parceldelivery_cv2x.h"

#ifdef NS_V2X_BUILD
#include <algorithm>
#include <random>

// ns3
#include "ns3/core-module.h"
#include <ns3/config-store.h>
#include <ns3/mobility-module.h>
#include <ns3/internet-module.h>
#include "ns3/lte-helper.h"
#include "ns3/lte-module.h"
#include <ns3/buildings-helper.h>
#include "ns3/ipv4-global-routing-helper.h"

// LIMoSim ns3
#include "ns3/callbacks.h"
#include "ns3/ns3setuphelpers.h"
#include "ns3/limosimmobilitymodel.h"
#include "ns3/strategicmodels/ns3_truckdeliverystrategy.h"
#include "ns3/behaviors/ns3_behavior_delivery.h"
#include "ns3/applications/udptransciever.h"
#include "ns3/applications-module.h"
#include "ns3/tags/timestamptag.h"
#include "ns3/behaviors/ns3_behavior_delivery.h"

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
namespace CV2X {

using namespace ns3;
using Node = ns3::Node;

static int total_count_rx=0;
static int total_count_tx=0;

static bool persistence=false;

static Time interval;

static uint32_t nbcars;
static uint32_t nbUAVs;

static int cam_length;


// Output
static std::string simtime = "log_simtime_v2x.csv";
static std::string data = "log_data_v2x.csv";
static std::string connections = "log_connections_v2x.csv";
static std::string positions = "log_positions_v2x.csv";


static Ptr<OutputStreamWrapper> log_connections;
static Ptr<OutputStreamWrapper> log_simtime;
static Ptr<OutputStreamWrapper> log_positions;
static Ptr<OutputStreamWrapper> log_data;

// Global variables
static uint32_t ctr_totRx = 0; 	// Counter for total received packets
static uint32_t ctr_totTx = 0; 	// Counter for total transmitted packets
static uint16_t lenCam;
static double baseline= 150.0;        // Baseline distance in meter (150m for urban, 320m for freeway)

// Responders users
static NodeContainer ueVeh;
static NodeContainer enbVeh;

static uint16_t nodeIdOffset = 1;

inline std::string exampleName() {
    return "ParcelDelivery_CV2X";
}
inline bool monitorPerformance() {
    return false;
}

NS_LOG_COMPONENT_DEFINE (exampleName());

void
PrintStatus (uint32_t s_period, Ptr<OutputStreamWrapper> log_simtime)
{
    if (ctr_totRx > ctr_totTx)
    {
        ctr_totRx = ctr_totTx;
    }
    *log_simtime->GetStream() << Simulator::Now ().GetSeconds ()
                              << ";" << ctr_totRx << ";" << ctr_totTx << ";"
                              << (double) ctr_totRx / ctr_totTx << std::endl;

    std::cout << Simulation::getInstance()->getName() << "_r"
              << Simulation::getInstance()->getRunCount()
              << " t=" <<  Simulator::Now().GetSeconds()
              << "\t Rx/Tx="<< ctr_totRx << "/" << ctr_totTx
              << "\t PRR=" << (double) ctr_totRx / ctr_totTx << std::endl;
    Simulator::Schedule(Seconds(s_period), &PrintStatus, s_period,log_simtime);
}

void
SidelinkV2xAnnouncementMacTrace(Ptr<Socket> socket)
{
    Ptr <Node> node = socket->GetNode();
    uint32_t id = node->GetId();
//    std::cout << "SidelinkV2xAnnouncementMacTrace node id: " << id << std::endl;
    uint32_t simTime = Simulator::Now().GetSeconds();
    Ptr<ns3::MobilityModel> posMobility = node->GetObject<ns3::MobilityModel>();
    Vector posTx = posMobility->GetPosition();

    if (id >= nbcars + nodeIdOffset) {
        //current node is UAV -> rcv is Car

        // check for distance to transmitter
        // Get receiver node
        Ptr<Node> rcvNode = enbVeh.Get(0);
        std::cout << "sending to car node " << rcvNode->GetId() << std::endl;
        Ptr<ns3::MobilityModel> mob = rcvNode->GetObject<ns3::MobilityModel>();
        Vector posRx = mob->GetPosition();

        double distance = sqrt(pow((posTx.x - posRx.x),2.0)+pow((posTx.y - posRx.y),2.0));
        if  (distance > 0 && distance <= baseline)
        {
            ctr_totTx++;
            UI::AnimationUtils::animateTransmission(std::to_string(node->GetId()));
        }

        // Generate CAM
        std::ostringstream msgCam;
        uint16_t packetSize = lenCam;

        msgCam << id-1 << "\t"
               << simTime << "\t"
               << ctr_totTx << "\t"
               << static_cast<int>(posTx.x)<< "\t"
               << static_cast<int>(posTx.y)<< '\0';
//        Ptr<Packet> packet = Create<Packet>((uint8_t*)msgCam.str().c_str(),packetSize);
//        std::cout << msgCam.str().c_str() << std::endl;
        Ptr<Packet> packet = Create<Packet>(reinterpret_cast<const uint8_t*>(msgCam.str().c_str()),packetSize);
        Tags::TimestampTag timestamp;
        timestamp.SetTimestamp (Simulator::Now());
        packet->AddByteTag(timestamp);
        socket->Send(packet);
//        std::cout << "finished sending" << std::endl;
        return;
    } else {
        // current node is Car -> rcv is UAV
        // cars do not send
        return;
    }
}

static void
ReceivePacket(Ptr<Socket> socket)
{
    Ptr<Node> node = socket->GetNode();
    Ptr<ns3::MobilityModel> posMobility = node->GetObject<ns3::MobilityModel>();
    Vector posRx = posMobility->GetPosition();
    Ptr<Packet> packet = socket->Recv ();
    uint8_t *buffer = new uint8_t[packet->GetSize()];
    packet->CopyData(buffer,packet->GetSize());
    std::string s = std::string((char*)buffer);

    size_t pos = 0;
    std::string copy = s;
    std::string token;
    int posTx_x;
    int posTx_y;
    for (int i = 0; i < 4; i++)
    {
        if (copy.find("\t") != std::string::npos)
        {
            pos = copy.find("\t");
            token = copy.substr(0,pos);
            if(i == 3)
            {
                posTx_x = atoi(token.c_str());
            }
            copy.erase (0,pos+1);
        }
    }
    posTx_y = atoi(copy.c_str());


    UI::AnimationUtils::animateReception(std::to_string(node->GetId()));
    uint32_t id = node->GetId();
    double simTime = Simulator::Now().GetSeconds();
    ctr_totRx++;
    total_count_rx++;

    // TODO use SeqTS in header instead
    Tags::TimestampTag timestamp;
    double delay_ms = 0;
    if (packet->FindFirstMatchingByteTag(timestamp)){
        Time tx = timestamp.GetTimestamp();
        delay_ms = (Simulator::Now() - tx).GetMilliSeconds();
    }

    double pdr = ctr_totTx * 1.0 / ctr_totRx;
    double datarate = ctr_totRx * packet->GetSize() * 8 / ( 1024*1024*Simulator::Now().GetSeconds());
    *log_data->GetStream() << ctr_totRx << "\t"
                           << simTime << "\t"
                           << id-1 << "\t"
                           << s << "\t"
                           << packet->GetSize() << "\t"
                           << delay_ms << "\t"
                           << pdr << "\t"
                           << datarate << std::endl;
}

void setup(uint16_t _runCount,
           uint16_t _numOfUAVs,
           uint16_t _numOfDeliveries)
{
    using namespace delivery;
    using namespace Behaviors;

    LogComponentEnable(exampleName().c_str(), ns3::LOG_LEVEL_INFO);
    LogComponentEnable("UdpTransciever",  ns3::LOG_LEVEL_INFO);
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



    bool disableDl = false;
    bool disableUl = false;
    bool disableUavReports = false;
    bool obstacleShadowing = true;

    uint32_t dlEarfcn = 100;
    uint32_t ulEarfcn = 18100;
    uint32_t dlResBlck = 100;
    uint32_t ulResBlck = 100;

    bool showProgress = true;

    // Initialize some values
    uint16_t simTime = 1;                      // Simulation time in seconds
    uint32_t numVeh = 1;                    // Number of vehicles
    lenCam = 190;                           // Length of CAM message in bytes [50-300 Bytes]
    double ueTxPower = 23.0;                // Transmission power in dBm
    double probResourceKeep = 0.0;          // Probability to select the previous resource again [0.0-0.8]
    uint32_t mcs = 20;                      // Modulation and Coding Scheme
    bool harqEnabled = false;               // Retransmission enabled
    bool adjacencyPscchPssch = true;        // Subchannelization scheme
    bool partialSensing = false;            // Partial sensing enabled (actual only partialSensing is false supported)
    uint16_t sizeSubchannel = 10;           // Number of RBs per subchannel
    uint16_t numSubchannel = 3;             // Number of subchannels per subframe
    uint16_t startRbSubchannel = 0;         // Index of first RB corresponding to subchannelization
    uint16_t pRsvp = 100;				    // Resource reservation interval
    uint16_t t1 = 4;                        // T1 value of selection window
    uint16_t t2 = 100;                      // T2 value of selection window
    uint16_t slBandwidth;                   // Sidelink bandwidth
    std::string tracefile;                  // Name of the tracefile
    uint16_t sidelinkPeriod = 1000;
    bool buildings = true;
    bool verbose = true;            // Print time progress

    ctr_totRx = 0;
    ctr_totTx = 0;
    total_count_rx = 0;
    total_count_tx = 0;

    srand(static_cast<unsigned int>(time(nullptr)));
    uint seed = static_cast<uint>(rand());
    RngSeedManager::SetSeed(seed);
    RngSeedManager::SetRun(static_cast<uint>(runCount));


    AsciiTraceHelper ascii;
    data = "../results/Sl_" +
            Simulation::getInstance()->getName() + "_" +
            std::to_string(runCount) +
            ".txt";
    log_simtime = ascii.CreateFileStream(simtime);
    log_data = ascii.CreateFileStream(data);
    log_connections = ascii.CreateFileStream(connections);
    log_positions = ascii.CreateFileStream(positions);
    nbcars = 1;
    nbUAVs = delivererCount;

    // Set the UEs power in dBm
    Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue (ueTxPower));
    Config::SetDefault ("ns3::LteUePhy::RsrpUeMeasThreshold", DoubleValue (-10.0));
    // Enable V2X communication on PHY layer
    Config::SetDefault ("ns3::LteUePhy::EnableV2x", BooleanValue (true));

    // Set power
    Config::SetDefault ("ns3::LteUePowerControl::Pcmax", DoubleValue (ueTxPower));
    Config::SetDefault ("ns3::LteUePowerControl::PsschTxPower", DoubleValue (ueTxPower));
    Config::SetDefault ("ns3::LteUePowerControl::PscchTxPower", DoubleValue (ueTxPower));

    if (adjacencyPscchPssch)
    {
        slBandwidth = sizeSubchannel * numSubchannel;
    }
    else
    {
        slBandwidth = (sizeSubchannel+2) * numSubchannel;
    }

    // Configure for UE selected
    Config::SetDefault ("ns3::LteUeMac::UlBandwidth", UintegerValue(slBandwidth));
    Config::SetDefault ("ns3::LteUeMac::EnableV2xHarq", BooleanValue(harqEnabled));
    Config::SetDefault ("ns3::LteUeMac::EnableAdjacencyPscchPssch", BooleanValue(adjacencyPscchPssch));
    Config::SetDefault ("ns3::LteUeMac::EnablePartialSensing", BooleanValue(partialSensing));
    Config::SetDefault ("ns3::LteUeMac::SlGrantMcs", UintegerValue(mcs));
    Config::SetDefault ("ns3::LteUeMac::SlSubchannelSize", UintegerValue (sizeSubchannel));
    Config::SetDefault ("ns3::LteUeMac::SlSubchannelNum", UintegerValue (numSubchannel));
    Config::SetDefault ("ns3::LteUeMac::SlStartRbSubchannel", UintegerValue (startRbSubchannel));
    Config::SetDefault ("ns3::LteUeMac::SlPrsvp", UintegerValue(pRsvp));
    Config::SetDefault ("ns3::LteUeMac::SlProbResourceKeep", DoubleValue(probResourceKeep));
    Config::SetDefault ("ns3::LteUeMac::SelectionWindowT1", UintegerValue(t1));
    Config::SetDefault ("ns3::LteUeMac::SelectionWindowT2", UintegerValue(t2));
//    Config::SetDefault ("ns3::LteUeMac::EnableExcludeSubframe", BooleanValue(excludeSubframe));



    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults ();

    // Network setup
    NS_LOG_INFO ("Creating nodes.");
    NodeContainer enbNodes;
    enbNodes.Create(1);
    NodeContainer LIMoSimNodes;
    LIMoSimNodes.Create (delivererCount +1);
    NodeContainer ueAllNodes;

    NS_LOG_INFO ("Creating helpers...");
    // EPC
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    Ptr<Node> pgw = epcHelper->GetPgwNode();

    // LTE Helper
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    lteHelper->SetEpcHelper(epcHelper);
    lteHelper->DisableNewEnbPhy(); // Disable eNBs for out-of-coverage modelling

    // V2X
    Ptr<LteV2xHelper> lteV2xHelper = CreateObject<LteV2xHelper> ();
    lteV2xHelper->SetLteHelper (lteHelper);

    // Configure eNBs' antenna parameters before deploying them.
    lteHelper->SetEnbAntennaModelType ("ns3::NistParabolic3dAntennaModel");

    lteHelper->SetAttribute ("UseSameUlDlPropagationCondition", BooleanValue(true));
    Config::SetDefault ("ns3::LteEnbNetDevice::UlEarfcn", StringValue ("54990"));
//    lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::CniUrbanmicrocellPropagationLossModel"));
    if (obstacleShadowing) {
        lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::DeterministicObstacleShadowingSpectrumPropagationLossModel"));
    } else {
        lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::HybridBuildingsPropagationLossModel"));
        lteHelper->SetPathlossModelAttribute ("ShadowSigmaOutdoor", DoubleValue (0.0));
        lteHelper->SetPathlossModelAttribute ("ShadowSigmaIndoor", DoubleValue (0.0));
        lteHelper->SetPathlossModelAttribute ("ShadowSigmaExtWalls", DoubleValue (0.0));
    }


    // Install mobility models on mobile nodes
    MobilityHelper mobilityHelper;
    // truck
    mobilityHelper.SetMobilityModel("LIMoSim::NS3::LIMoSimMobility",
                                    "VehicleId", StringValue("T0"));
    mobilityHelper.Install(LIMoSimNodes.Get(0));
    // uavs
    for (uint uavIndex = 0; uavIndex < delivererCount; uavIndex++) {
        mobilityHelper.SetMobilityModel("LIMoSim::NS3::LIMoSimMobility",
                                        "VehicleId", StringValue("U" + std::to_string(uavIndex)));
        mobilityHelper.Install(LIMoSimNodes.Get(uavIndex + 1));
    }
    // base station
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
    positionAlloc->Add (Vector(0, 0, 60)); // ENB positions
    mobilityHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityHelper.SetPositionAllocator(positionAlloc);
    mobilityHelper.Install(enbNodes);

    // Install Service
    NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice(enbNodes);

    enbVeh.Add(LIMoSimNodes.Get(0));
    for (uint i = 0; i < delivererCount; i++){
        ueVeh.Add(LIMoSimNodes.Get(i+1));
    }
    ueAllNodes.Add(enbVeh);
    ueAllNodes.Add(ueVeh);
    std::cout << "nodeid: " << enbVeh.Get(0)->GetId() << std::endl;

    // Required to use NIST 3GPP model
    BuildingsHelper::Install (enbNodes);
    BuildingsHelper::Install (ueAllNodes);
    BuildingsHelper::MakeMobilityModelConsistent ();

    VehicleManager *vehicleManager = VehicleManager::getInstance();
    UI::Data::UIDataManager *uidm = UI::Data::UIDataManager::getInstance();
    using namespace delivery;

    mobility::car::DeliveryTruck *truck = vehicleManager->createDeliveryTruck(
                "T0",
                delivererCount,
                deliveryList
                );
    uidm->getVehicleData(truck->getId())->getShape()->setColor("red");
    auto truckDeliveryStrategy = CreateObjectWithAttributes<StrategicModels::NS3_TruckDeliveryStrategy>(
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
//        behavior->initializeMobility();
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


    std::vector<NodeContainer> nodePairs;
    for (uint16_t i = 0; i < delivererCount; i++) {
        NodeContainer pair;
        pair.Add(enbVeh.Get(0));
        pair.Add(ueVeh.Get(i));
        nodePairs.push_back(pair);
    }

    // Install LTE devices to all UEs
    NS_LOG_INFO ("Installing UE's network devices ...");
    lteHelper->SetAttribute ("UseSidelink", BooleanValue (true));
    std::vector<NetDeviceContainer> ueRespDevPairs;
    NetDeviceContainer ueDevs;
    NetDeviceContainer ueTruck = lteHelper->InstallUeDevice(enbVeh.Get(0));
    ueDevs.Add(ueTruck);
    for (uint16_t i = 0; i < delivererCount; i++) {
        NetDeviceContainer ueRespondersDevsPair;
        NetDeviceContainer ueUav = lteHelper->InstallUeDevice(nodePairs.at(i).Get(1));
        ueRespondersDevsPair.Add(ueTruck);
        ueRespondersDevsPair.Add(ueUav);
        ueDevs.Add(ueUav);
        // quick hack
//        ueRespondersDevsPair.Add(ueRespondersDevsPair.Get(1));
        ueRespDevPairs.push_back(ueRespondersDevsPair);
    }

    // Install the IP stack on the UEs
    NS_LOG_INFO ("Installing IP stack...");
    InternetStackHelper internet;
    internet.Install (ueAllNodes);

    // Assign IP address to UEs
    NS_LOG_INFO ("Allocating IP addresses and setting up network route...");
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address (ueDevs);
    Ipv4StaticRoutingHelper ipv4RoutingHelper;

    for (uint32_t u = 0; u < ueAllNodes.GetN (); ++u)
    {
        Ptr<Node> ueNode = ueAllNodes.Get (u);
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
        ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

    NS_LOG_INFO ("Attaching UE's to LTE network...");
    // Attach each UE to the best available eNB
    lteHelper->Attach (ueDevs);

    NS_LOG_INFO ("Creating sidelink groups...");
    // Create groups
    std::vector<std::vector<NetDeviceContainer>> txGroupsPairs;
    for (uint16_t i = 0; i < delivererCount; i++) {
        // quick hack consequence: fictional third member in pair.
        std::vector<NetDeviceContainer> txGroups = lteV2xHelper->AssociateForV2xBroadcast(ueRespDevPairs.at(i), 2);
//        std::vector<NetDeviceContainer> txGroups = lteV2xHelper->AssociateForV2xBroadcast(ueRespDevPairs.at(i), 3);
        txGroupsPairs.push_back(txGroups);
        std::cout << "Sidelink group #" << i << std::endl;
        lteV2xHelper->PrintGroups(txGroups);
    }

    //    std::vector<NetDeviceContainer> txGroups;
    //    txGroups = lteV2xHelper->AssociateForV2xBroadcast(ueRespondersDevs, numVeh);
    //    lteV2xHelper->PrintGroups(txGroups);


    //compute average number of receivers associated per transmitter and vice versa
    double totalRxs = 0;
    std::map < uint32_t, uint32_t> txPerUeMap;
    std::map < uint32_t, uint32_t> groupsPerUe;
    // Application Setup for Responders
    std::vector<uint32_t> groupL2Addresses;
    uint32_t groupL2Address = 0x00;
    Ipv4AddressGenerator::Init(Ipv4Address ("225.0.0.0"), Ipv4Mask("255.0.0.0"));
    Ipv4Address clientRespondersAddress = Ipv4AddressGenerator::NextAddress (Ipv4Mask ("255.0.0.0"));


    for (uint16_t i = 0; i < delivererCount; i++) {
        std::vector<NetDeviceContainer> txGroups = txGroupsPairs.at(i);
//        connectGroupsInUi(txGroups);
        std::vector < NetDeviceContainer >::iterator gIt;
        for (gIt = txGroups.begin() ; gIt != txGroups.end() ; gIt++)
        {
            uint32_t numDevs = gIt->GetN ();

              totalRxs += numDevs - 1;
              uint32_t nId;

            for (uint32_t i = 1; i < numDevs; i++)
            {
                  nId = gIt->Get (i)->GetNode ()->GetId ();
                  txPerUeMap[nId]++;
            }
        }

        double totalTxPerUe = 0;
        std::map < uint32_t, uint32_t>::iterator mIt;
    //    std::cout << "Tx per UE:" << std::endl;
        for (mIt = txPerUeMap.begin (); mIt != txPerUeMap.end() ; mIt++)
          {
            totalTxPerUe+= mIt->second;
    //        std::cout << mIt->first << " " << mIt->second << std::endl;
            groupsPerUe [mIt->second]++;
          }
        lteV2xHelper->PrintGroups (txGroups, log_connections);
//        std::cout << "Associated Groups per Rx UE" << std::endl;
//        for (mIt = groupsPerUe.begin (); mIt != groupsPerUe.end (); mIt++)
//        {
//            std::cout << mIt->first << " " << mIt->second << std::endl;
//        }



        NS_LOG_INFO ("Installing applications...");

        uint16_t application_port = 8000; // Application port to TX/RX
        NetDeviceContainer activeTxUes;


        for(gIt=txGroups.begin(); gIt != txGroups.end(); gIt++)
        {
            // Create Sidelink bearers
            // Use Tx for the group transmitter and Rx for all the receivers
            // Split Tx/Rx

            NetDeviceContainer txUe ((*gIt).Get(0));
            activeTxUes.Add(txUe);
            NetDeviceContainer rxUes = lteV2xHelper->RemoveNetDevice ((*gIt), txUe.Get (0));
            Ptr<LteSlTft> tft = Create<LteSlTft> (LteSlTft::TRANSMIT, clientRespondersAddress, groupL2Address);
            lteV2xHelper->ActivateSidelinkBearer (Seconds(0.0), txUe, tft);
            tft = Create<LteSlTft> (LteSlTft::RECEIVE, clientRespondersAddress, groupL2Address);
            lteV2xHelper->ActivateSidelinkBearer (Seconds(0.0), rxUes, tft);

            std::cout << "Created group L2Address=" << groupL2Address << " IPAddress=";
            clientRespondersAddress.Print(std::cout);
            std::cout << std::endl;

            //Individual Socket Traffic Broadcast everyone
            Ptr<Socket> host = Socket::CreateSocket(txUe.Get(0)->GetNode(),TypeId::LookupByName ("ns3::UdpSocketFactory"));
            host->Bind();
            host->Connect(InetSocketAddress(clientRespondersAddress,application_port));
            host->SetAllowBroadcast(true);
            host->ShutdownRecv();

            /*Ptr<LteUeRrc> ueRrc = DynamicCast<LteUeRrc>( txUe.Get (0)->GetObject<LteUeNetDevice> ()->GetRrc () );
            ueRrc->TraceConnectWithoutContext ("SidelinkV2xMonitoring", MakeBoundCallback (&SidelinkV2xMonitoringTrace, stream));
            oss << txUe.Get(0) ->GetObject<LteUeNetDevice>()->GetImsi();
            Ptr<LteUePhy> uePhy = DynamicCast<LteUePhy>( txUe.Get (0)->GetObject<LteUeNetDevice> ()->GetPhy () );
            uePhy->TraceConnect ("SidelinkV2xAnnouncement", oss.str() ,MakeBoundCallback (&SidelinkV2xAnnouncementPhyTrace, stream1));*/
            //uePhy->TraceConnectWithoutContext ("SidelinkV2xAnnouncement", MakeBoundCallback (&SidelinkV2xAnnouncementPhyTrace, host));
            Ptr<LteUeMac> ueMac = DynamicCast<LteUeMac>( txUe.Get (0)->GetObject<LteUeNetDevice> ()->GetMac () );
            ueMac->TraceConnectWithoutContext ("SidelinkV2xAnnouncement", MakeBoundCallback (&SidelinkV2xAnnouncementMacTrace, host));
            //ueMac->TraceConnect ("SidelinkV2xAnnouncement", oss.str() ,MakeBoundCallback (&SidelinkV2xAnnouncementMacTrace, stream2));

            Ptr<Socket> sink = Socket::CreateSocket(txUe.Get(0)->GetNode(),TypeId::LookupByName ("ns3::UdpSocketFactory"));
            sink->Bind(InetSocketAddress (Ipv4Address::GetAny (), application_port));
            sink->SetRecvCallback (MakeCallback (&ReceivePacket));

            //store and increment addresses
            groupL2Addresses.push_back (groupL2Address);
            groupL2Address++;
            clientRespondersAddress = Ipv4AddressGenerator::NextAddress (Ipv4Mask ("255.0.0.0"));
        }
    }

    NS_LOG_INFO ("Creating Sidelink Configuration...");
    Ptr<LteUeRrcSl> ueSidelinkConfiguration = CreateObject<LteUeRrcSl>();
    ueSidelinkConfiguration->SetSlEnabled(true);
    ueSidelinkConfiguration->SetV2xEnabled(true);

    LteRrcSap::SlV2xPreconfiguration preconfiguration;
    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.carrierFreq = 54890;
    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.slBandwidth = slBandwidth;

    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.nbPools = 1;
    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.nbPools = 1;

//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0].slSubframeV2x.bitmap = std::bitset<20> (0xFFFFF);
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0].adjacencyPscchPssch.adjacency = adjacencyPscchPssch;
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0].sizeSubchannel = LteRrcSap::SizeSubchannelFromInt (sizeSubchannel);
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0].numSubchannel = LteRrcSap::NumSubchannelFromInt (numSubchannel);
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0].startRbSubchannel.startRb = startRbSubchannel;
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0].dataTxParameters.p0 = -4;
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0].dataTxParameters.alpha = LteRrcSap::SlTxParameters::al09;

//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.nbPools = 1;
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0].slSubframeV2x.bitmap = std::bitset<20> (0xFFFFF);
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0].adjacencyPscchPssch.adjacency = adjacencyPscchPssch;
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0].sizeSubchannel = LteRrcSap::SizeSubchannelFromInt (sizeSubchannel);
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0].numSubchannel = LteRrcSap::NumSubchannelFromInt (numSubchannel);
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0].startRbSubchannel.startRb = startRbSubchannel;
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0].dataTxParameters.p0 = -4;
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0].dataTxParameters.alpha = LteRrcSap::SlTxParameters::al09;

    SlV2xPreconfigPoolFactory pFactory;
    pFactory.SetHaveUeSelectedResourceConfig (true);
    pFactory.SetSlSubframe (std::bitset<20> (0xFFFFF));
    pFactory.SetAdjacencyPscchPssch (adjacencyPscchPssch);
    pFactory.SetSizeSubchannel (sizeSubchannel);
    pFactory.SetNumSubchannel (numSubchannel);
    pFactory.SetStartRbSubchannel (startRbSubchannel);
    pFactory.SetStartRbPscchPool (0);
    pFactory.SetDataTxP0 (-4);
    pFactory.SetDataTxAlpha (0.9);
    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0] = pFactory.CreatePool ();
    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0] = pFactory.CreatePool ();
    ueSidelinkConfiguration->SetSlV2xPreconfiguration (preconfiguration);


    // Print Configuration
    *log_data->GetStream() << "RxPkts\tRxTime\tRxId\tTxId\tTxTime\tTxPkts\tPosX\tPosy\tBytes\tDelay\tPDR\tDatarate" << std::endl;

    NS_LOG_INFO ("Installing Sidelink Configuration...");
    lteHelper->InstallSidelinkV2xConfiguration (ueDevs, ueSidelinkConfiguration);

    NS_LOG_INFO ("Enabling LTE traces...");
    lteHelper->EnableTraces();
    Ptr<RadioBearerStatsCalculator> pdcpStats = lteHelper->GetPdcpStats ();
    pdcpStats->SetDlPdcpOutputFilename("../results/DlPdcpStats_" + exampleName() + "_" + std::to_string(runCount) + ".txt");

    *log_simtime->GetStream() << "Simtime; Total Rx; Total Tx; PRR" << std::endl;
    // Display simulator time progress
    Simulator::Schedule(Seconds(1), &PrintStatus, 1, log_simtime);


    VehicleManager::getInstance()->enableMobilityBroadcastHelper();



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

    Simulator::Stop(Seconds(900));

//    setupIpv4TrafficAnimation();
    registerStaticNodes();
    startSimulation();
}


} // namespace CV2X
} // namespace ParcelDelivery
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim
#endif
