#include "parceldelivery_lte.h"

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
#include <ns3/point-to-point-helper.h>

// LIMoSim ns3
#include "ns3/callbacks.h"
#include "ns3/ns3setuphelpers.h"
#include "ns3/limosimmobilitymodel.h"
#include "ns3/strategicmodels/ns3_truckdeliverystrategy.h"
#include "ns3/behaviors/ns3_behavior_delivery.h"
#include "ns3/applications/udptransciever.h"
#include "ns3/applications-module.h"

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
namespace LTE {

using namespace ns3;
using Node = ns3::Node;

inline std::string exampleName() {
    return "ParcelDelivery_LTE";
}
inline bool monitorPerformance() {
    return false;
}

NS_LOG_COMPONENT_DEFINE (exampleName());

void receiveUavReport(Ptr<const Packet> _packet, const Address & _address) {
    SeqTsHeader sq;
    _packet->PeekHeader(sq);
    double delay = (Simulator::Now() - sq.GetTs()).GetMilliSeconds();
    std::cout << "received UAV report from " << Ipv4Address::ConvertFrom(_address) << "with delay " << delay << std::endl;
}

void setup(uint16_t _runCount,
           uint16_t _numOfUAVs,
           uint16_t _numOfDeliveries,
           uint16_t _numOfDisturbers,
           uint16_t _disturberInterPacketInterval_ms,
           uint16_t _disturberPacketSize_B)
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



    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults ();
    Time interPacketInterval = MilliSeconds(_disturberInterPacketInterval_ms);
    uint64_t packetSize = _disturberPacketSize_B; // in Bytes - ns3 max is 63KB
    bool disableDl = false;
    bool disableUl = false;
    bool disableUavReports = false;
    uint32_t dlEarfcn = 100;
    uint32_t ulEarfcn = 18100;
    uint32_t dlResBlck = 100;
    uint32_t ulResBlck = 100;
    bool showProgress = true;
    bool disturbersEnabled = false;
    bool obstacleShadowing = true;
    uint uavReportSize_B = 190;
    uint uavRepotPeriod_ms = 100;

    Config::SetDefault ("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue (false));
    Config::SetDefault ("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue (false));
    Config::SetDefault ("ns3::LteAmc::AmcModel", EnumValue (LteAmc::PiroEW2010));
    Config::SetDefault ("ns3::LteAmc::Ber", DoubleValue (0.00005));
    Config::SetDefault ("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue(ulResBlck));
    Config::SetDefault ("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue(dlResBlck));
    Config::SetDefault ("ns3::LteEnbNetDevice::UlEarfcn", UintegerValue(ulEarfcn));
    Config::SetDefault ("ns3::LteEnbNetDevice::DlEarfcn", UintegerValue(dlEarfcn));
    Config::SetDefault ("ns3::PhyStatsCalculator::DlRsrpSinrFilename",
                        StringValue("../results/DlRsrpSinrStats_" +
                                    Simulation::getInstance()->getName() + "_" +
                                    std::to_string(runCount) + ".txt"));

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
    if (obstacleShadowing) {
        lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::DeterministicObstacleShadowingSpectrumPropagationLossModel"));
    } else {
        lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::FriisSpectrumPropagationLossModel"));
    }

    Config::SetDefault ("ns3::LteUePhy::EnableUplinkPowerControl", BooleanValue (false));
    lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler");
    lteHelper->SetSchedulerAttribute ("UlCqiFilter", EnumValue (FfMacScheduler::PUSCH_UL_CQI));

    srand(static_cast<unsigned int>(time(nullptr)));
    uint seed = static_cast<uint>(rand());
    RngSeedManager::SetSeed (seed);
    RngSeedManager::SetRun (static_cast<uint>(runCount));


    // Network setup

    NS_LOG_INFO ("Creating nodes.");
    NodeContainer enbNodes;
    enbNodes.Create(1);
    NodeContainer LIMoSimNodes;
    LIMoSimNodes.Create (delivererCount +1);
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create (1);

    Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
    lteHelper->SetEpcHelper (epcHelper);

    Ptr<Node> pgw = epcHelper->GetPgwNode ();

    // Create a single RemoteHost
    Ptr<Node> remoteHost = remoteHostContainer.Get (0);


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

    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
    positionAlloc->Add (Vector(0, 0, 60)); // ENB positions
    mobilityHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityHelper.SetPositionAllocator(positionAlloc);
    mobilityHelper.Install(enbNodes);

    InternetStackHelper internet;

    NS_LOG_INFO ("Installing WiFi and Internet stack.");
    // Create the Internet
    // IP Stack for remote host
    internet.Install (remoteHostContainer);
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
    p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
    p2ph.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (10)));
    NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
    // interface 0 is localhost, 1 is the p2p device
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
    remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

    // Install LTE Devices to the nodes
    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(LIMoSimNodes);

    // Install the IP stack on the UEs
    internet.Install(LIMoSimNodes);
    Ipv4AddressHelper ipAddrs;
    ipAddrs.SetBase ("192.168.0.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = epcHelper->AssignUeIpv4Address(ueLteDevs);

    // Attach all UEs to first eNodeB
    for (uint16_t i = 0; i <= delivererCount; i++)
    {
        lteHelper->Attach (ueLteDevs.Get(i), enbLteDevs.Get(0));
    }



    // Print IMSIs
    std::cout << "UE IMSIs" << std::endl;
    std::cout << "Truck: " << (ns3::DynamicCast<LteUeNetDevice>(ueLteDevs.Get(0)))->GetImsi()
              << std::endl;
    for (uint uavIndex = 0; uavIndex < delivererCount; uavIndex++) {
        std::cout << "UAV"<<uavIndex << ": "
                  << (ns3::DynamicCast<LteUeNetDevice>(ueLteDevs.Get(uavIndex+1)))->GetImsi()
                  << std::endl;
    }


//    internet.EnableAsciiIpv4All("../results/"+ Simulation::getInstance()->getName());
//    internet.EnablePcapIpv4All("../results/"+ Simulation::getInstance()->getName());


    VehicleManager *vehicleManager = VehicleManager::getInstance();
    UI::Data::UIDataManager *uidm = UI::Data::UIDataManager::getInstance();
    using namespace delivery;

    vehicleManager->enableMobilityBroadcastHelper();

//    StringVector uavAddressList;
//    for (uint uavIndex = 1; uavIndex < interfaces.GetN(); uavIndex++) {
//        std::ostringstream os;
//        os << interfaces.GetAddress(uavIndex);
//        uavAddressList.push_back(os.str());
//    }

//    std::cout << "UAV IP list:" << std::endl;
//    for (uint uavIndex = 0; uavIndex < delivererCount; uavIndex++) {
//        std::cout << "UAV" <<uavIndex <<": " << uavAddressList.at(uavIndex) <<std::endl;
//    }

//    StringVector truckAddressList;
//    {
//        std::ostringstream os;
//        os << interfaces.GetAddress(0);
//        truckAddressList.push_back(os.str());
//    }
//    std::cout << "Truck IP list:" << std::endl;
//    std::cout << "Truck" <<0 <<": " << truckAddressList.at(0) <<std::endl;;

    mobility::car::DeliveryTruck *truck = vehicleManager->createDeliveryTruck(
                "T0",
                delivererCount,
                deliveryList
                );


//    uint port = 8080;
    uidm->getVehicleData(truck->getId())->getShape()->setColor("red");
    // CAUTION: The ns3 truck delivery strategy is an ns3 object
    // wrapped in a smart pointer so deletion of the reference object
    // could have unforeseen consequences.
    auto truckDeliveryStrategy = CreateObjectWithAttributes<StrategicModels::NS3_TruckDeliveryStrategy>(
                "TruckId", StringValue("T0")
                );
//    Ptr<UdpTransciever> truckTransciever = CreateObjectWithAttributes<UdpTransciever>(
//                "Port", UintegerValue(port)
//                );
//    truckTransciever->setDestinationAddresses(uavAddressList);
    truckDeliveryStrategy->initializeMobility("T0");
//    LIMoSimNodes.Get(0)->AddApplication(truckTransciever);
//    LIMoSimNodes.Get(0)->AggregateObject(truckTransciever);
    LIMoSimNodes.Get(0)->AddApplication(truckDeliveryStrategy);
    truckDeliveryStrategy->SetStartTime(Seconds(0));
//    truckTransciever->SetStartTime(Seconds(0));
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
//        LIMoSimNodes.Get(1)->GetObject<ns3::MobilityModel>()->SetPosition(toNS3Vector(uav->getPosition()));
        uav->setBehavior(&*behavior);
        behavior->setTruckDeliveryStrategy(&*truckDeliveryStrategy);
        behavior->setCommunicationsEnabled(false);
        uavDeliveryBehaviors.push_back(&*behavior);

//        Ptr<UdpTransciever> uavTransciever = CreateObjectWithAttributes<UdpTransciever>(
//                    "Port", UintegerValue(port)
//                    );
//        uavTransciever->setDestinationAddresses(truckAddressList);
//        LIMoSimNodes.Get(i+1)->AddApplication(uavTransciever);
//        LIMoSimNodes.Get(1)->AggregateObject(uavTransciever);
        LIMoSimNodes.Get(i+1)->AddApplication(behavior);
        behavior->SetStartTime(Seconds(0));
//        uavTransciever->SetStartTime(Seconds(0));
    }
    truckDeliveryStrategy->setUavDeliveryBehaviors(uavDeliveryBehaviors);
    truckDeliveryStrategy->setCommunicationsEnabled(false);

    if (!disableUavReports) {
        // UAV -> Truck Traffic
        uint16_t reportPort = 1200;
        ApplicationContainer serverReportApps;
        ApplicationContainer clientReportApps;
        for (uint16_t uavIndex = 0; uavIndex < delivererCount; ++uavIndex) {
            // offset of 1 because first node is truck
            Ptr<Node> uavNode = LIMoSimNodes.Get(uavIndex+1);
            ++reportPort;

            PacketSinkHelper reportPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny(), reportPort));
            // Truck is packet sink
            serverReportApps.Add(reportPacketSinkHelper.Install(LIMoSimNodes.Get(0)));

            UdpClientHelper reportClient(interfaces.GetAddress(0), reportPort);
            reportClient.SetAttribute("Interval", TimeValue(MilliSeconds(uavRepotPeriod_ms)));
            reportClient.SetAttribute("MaxPackets", UintegerValue(1e6));
            reportClient.SetAttribute("PacketSize", UintegerValue(uavReportSize_B));

            clientReportApps.Add(reportClient.Install(uavNode));
        }

    //    uint appOffset = 2;
    //    for(uint16_t serverIndex = 0; serverIndex < serverReportApps.GetN(); serverIndex++) {
    //        Ptr<PacketSink> serverApp = serverReportApps.Get(serverIndex);
    //        std::ostringstream contextPath;
    //        contextPath << "/NodeList/"
    //                    << LIMoSimNodes.Get(0)->GetId()
    //                    << "/ApplicationList/"
    //                    << serverIndex + appOffset
    //                    << "/$ns3::PacketSink/Rx";
    //        Config::ConnectWithoutContext(contextPath.str(), MakeCallback(&receiveUavReport));
    //    }
        // Server must be started right away
        serverReportApps.Start (MilliSeconds (2));
        // Idealy clients should send report only when the UAV is on delivery
        // but starting and stopping applications during simulation is not possible
        clientReportApps.Start (MilliSeconds (5));
    }


    if (disturbersEnabled) {

        NodeContainer disturberNodes;
        disturberNodes.Create(_numOfDisturbers);


        // Set disturber nodes mobility
        Ptr<RandomDiscPositionAllocator> randomDiscPosAlloc;
        Ptr<RandomRectanglePositionAllocator> randomRecPosAlloc = CreateObject<RandomRectanglePositionAllocator> ();
        randomRecPosAlloc->SetX(CreateObjectWithAttributes<UniformRandomVariable>("Min", DoubleValue(0.0), "Max", DoubleValue(1000.0)));
        randomRecPosAlloc->SetY(CreateObjectWithAttributes<UniformRandomVariable>("Min", DoubleValue(-1000.0), "Max", DoubleValue(1000.0)));
        mobilityHelper.SetPositionAllocator(randomRecPosAlloc);
        mobilityHelper.Install(disturberNodes);

        // Install LTE UE Devices to the disturber nodes
        NetDeviceContainer disturberUeLteDevs = lteHelper->InstallUeDevice(disturberNodes);

        // Install internet on disturbers
        internet.Install(disturberNodes);

        // Assign IP to disturber UEs
        Ipv4InterfaceContainer disturberInterfaces = epcHelper->AssignUeIpv4Address(disturberUeLteDevs);
        // Set the default gateway for the disturber UEs to allow communication
        // with remote host
        for (uint32_t u = 0; u < disturberNodes.GetN (); ++u)
        {
            Ptr<Node> disturberUeNode = disturberNodes.Get (u);
            // Set the default gateway for the UE
            Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (disturberUeNode->GetObject<Ipv4> ());
            ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
        }


        // Attach disturber nodes to eNB
        for (uint16_t i = 0; i < _numOfDisturbers; i++)
        {
            lteHelper->Attach (disturberUeLteDevs.Get(i), enbLteDevs.Get(0));
        }

        // Install and start applications on disturber UEs and remote host
        uint16_t dlPort = 1100;
        uint16_t ulPort = 2000;
        uint16_t otherPort = 3000;
        ApplicationContainer clientApps;
        ApplicationContainer serverApps;
        for (uint32_t u = 0; u < disturberNodes.GetN (); ++u)
        {
            // Downlink remote host -> UE
            if (!disableDl)
            {
                PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
                serverApps.Add (dlPacketSinkHelper.Install (disturberNodes.Get(u)));

                UdpClientHelper dlClient (disturberInterfaces.GetAddress (u), dlPort);
                dlClient.SetAttribute ("Interval", TimeValue (interPacketInterval));
                dlClient.SetAttribute ("MaxPackets", UintegerValue (1000000));
                dlClient.SetAttribute ("PacketSize", UintegerValue(packetSize));
                clientApps.Add (dlClient.Install (remoteHost));
            }

            // Uplink remote host <- UE
            if (!disableUl)
            {
                ++ulPort;
                PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
                serverApps.Add (ulPacketSinkHelper.Install (remoteHost));

                UdpClientHelper ulClient (remoteHostAddr, ulPort);
                ulClient.SetAttribute ("Interval", TimeValue (interPacketInterval));
                ulClient.SetAttribute ("MaxPackets", UintegerValue (1000000));
                ulClient.SetAttribute ("PacketSize", UintegerValue(packetSize));
                clientApps.Add (ulClient.Install (disturberNodes.Get(u)));
            }
        }

        serverApps.Start (MilliSeconds (2));
        clientApps.Start (MilliSeconds (5));

    }



    Ptr<LteEnbNetDevice> lteEnbDev = enbLteDevs.Get (0)->GetObject<LteEnbNetDevice> ();
    Ptr<LteEnbPhy> enbPhy = lteEnbDev->GetPhy ();
    enbPhy->SetAttribute ("TxPower", DoubleValue (43.0));
    enbPhy->SetAttribute ("NoiseFigure", DoubleValue (5.0));

    // Set UEs' position and power
    for (uint i = 0; i < ueLteDevs.GetN(); i++)
    {
        Ptr<LteUeNetDevice> lteUeDev = ueLteDevs.Get (i)->GetObject<LteUeNetDevice> ();
        Ptr<LteUePhy> uePhy = lteUeDev->GetPhy ();
        uePhy->SetAttribute ("TxPower", DoubleValue (23.0));
        uePhy->SetAttribute ("NoiseFigure", DoubleValue (9.0));
    }

    // Simulation setup end
    std::string resultFolderPrefix = "";

    if (monitorPerformance()) {
        PerformanceMonitor::getInstance()->reset();
        PerformanceMonitor::getInstance()->setupWithExport("../results"+ resultFolderPrefix + "/Performance_" +
                                                           Simulation::getInstance()->getName() + "_" +
                                                           std::to_string(runCount) +
                                                           ".csv");
    }
    lteHelper->EnablePdcpTraces();
//    lteHelper->EnablePhyTraces();

    Ptr<RadioBearerStatsCalculator> pdcpStats = lteHelper->GetPdcpStats ();
    pdcpStats->SetDlPdcpOutputFilename("../results"+ resultFolderPrefix + "/DlPdcpStats_" +
                                       Simulation::getInstance()->getName() + "_" +
                                       std::to_string(runCount) + ".txt");
    pdcpStats->SetUlPdcpOutputFilename("../results"+ resultFolderPrefix + "/UlPdcpStats_" +
                                       Simulation::getInstance()->getName() + "_" +
                                       std::to_string(runCount) + ".txt");


    // Display simulator time progress
    if (showProgress)
    {
        uint32_t status_period = 1;
        Simulator::Schedule(Seconds(1),&Callbacks::PrintProgress, status_period);
    }

    Simulator::Stop(Seconds(900));

    setupIpv4TrafficAnimation();
    registerStaticNodes();
    startSimulation();
}



} // namespace LTE
} // namespace ParcelDelivery
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim
