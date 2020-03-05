#include "parceldelivery_wave.h"

#include <algorithm>
#include <random>

// ns3
#include "ns3/core-module.h"
#include <ns3/config-store.h>
#include <ns3/mobility-module.h>
#include <ns3/internet-module.h>
#include <ns3/yans-wifi-helper.h>
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/applications-module.h"

// LIMoSim ns3
#include "ns3/ns3setuphelpers.h"
#include "ns3/limosimmobilitymodel.h"
#include "ns3/strategicmodels/ns3_truckdeliverystrategy.h"
#include "ns3/behaviors/ns3_behavior_delivery.h"
#include "ns3/applications/udptransciever.h"
#include "ns3/tags/timestamptag.h"

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
namespace Wave {

using namespace ns3;

inline std::string exampleName() {
    return "ParcelDelivery_Wave";
}
inline bool monitorPerformance() {
    return false;
}

NS_LOG_COMPONENT_DEFINE (exampleName());

void ReceivePacketInSink(std::string _context, Ptr<const Packet> _packet, const Address& _address) {
    std::cout << "Received packet in sink" << std::endl;
//    Header header;
    SeqTsHeader seqTs;
    _packet->PeekHeader(seqTs);
    Time tx = seqTs.GetTs();
    std::cout << "packet delay: " << (Simulator::Now() - tx).GetMilliSeconds() << std::endl;
    std::cout << "packet size: " << _packet->GetSize() << std::endl;
}

void ReceivePacketCallback (std::string _context, Ptr< const Packet > _packet) {
    std::cout << "Received wifi packet at MAC Layer" << std::endl;
    Tags::TimestampTag timestamp;
    if(_packet->FindFirstMatchingByteTag(timestamp)) {
        Time tx = Simulator::Now() - timestamp.GetTimestamp();
        std::cout << "retrieved delay: " << tx.GetMicroSeconds() << std::endl;
    }

}

void SendPacketCallback (std::string _context, Ptr<const Packet> _packet) {
    std::cout << "Sending wifi packet at MAC Layer" << std::endl;
    Tags::TimestampTag timestamp;
    timestamp.SetTimestamp(Simulator::Now());
    _packet->AddByteTag(timestamp);
}

void setup(uint16_t _runCount,
           uint16_t _numOfUAVs,
           uint16_t _numOfDeliveries) {

    using namespace delivery;

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

    parser.addOptions({delivererCountOption, deliveryCountOption, usePredictionOption, deliveryListFileOption, droneInteractionModeOption});
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
    std::cout << "setting up ns3 simulation setup of " << exampleName() << " run count:" << _runCount <<std::endl;
    Simulation::getInstance()->setName(exampleName() + "_" +
                                       "dh" + std::to_string(deliveryListHash) + "_" +
                                       "nu" + std::to_string(delivererCount) + "_"+
                                       "nd" + std::to_string(deliveryCount) + "_" +
                                       "pr" + std::to_string(usePrediction) + "_" +
                                       "di" + std::to_string(droneInteractionMode))->setRunCount(_runCount);

    // Print simulation parameters

    std::cout << "Scenario params: " <<std::endl;
    std::cout << "- num of UAVs: " << delivererCount << std::endl;
    std::cout << "- num of deliveries: " << deliveryCount << std::endl;
    std::cout << "- drone interaction: " << droneInteractionMode << std::endl;



    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults ();

    // Network setup

    NS_LOG_INFO ("Creating nodes.");
    NodeContainer LIMoSimNodes;
    LIMoSimNodes.Create (2);

    NS_LOG_INFO ("Installing WiFi and Internet stack.");
    std::string phyMode ("OfdmRate6MbpsBW10MHz");
//    WifiHelper wifi;
    WifiMacHelper wifiMac;

    wifiMac.SetType ("ns3::AdhocWifiMac");
    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
//    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper();
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
//    wifiChannel.AddPropagationLoss("ns3::DeterministicObstacleShadowingSpectrumPropagationLossModel");
    wifiPhy.SetChannel (wifiChannel.Create ());
    // ns-3 supports generate a pcap trace
    wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
    NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
    Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
    wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                           "DataMode",StringValue (phyMode),
                                           "ControlMode",StringValue (phyMode));
//    wifi80211p.EnableLogComponents();
    NetDeviceContainer nodeDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, LIMoSimNodes);

    // Internet stack
    InternetStackHelper internet;
    internet.Install (LIMoSimNodes);
    Ipv4AddressHelper ipAddrs;
    ipAddrs.SetBase ("192.168.0.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = ipAddrs.Assign (nodeDevices);

//    internet.EnableAsciiIpv4All("../results/parcelDelivery");
//    internet.EnablePcapIpv4All("../results/"+ Simulation::getInstance()->getName());

    // Define mobility
    MobilityHelper mobilityHelper;
    mobilityHelper.SetMobilityModel("LIMoSim::NS3::LIMoSimMobility",
                                    "VehicleId", StringValue("T0"));
    mobilityHelper.Install(LIMoSimNodes.Get(0));
    mobilityHelper.SetMobilityModel("LIMoSim::NS3::LIMoSimMobility",
                                    "VehicleId", StringValue("U0"));
    mobilityHelper.Install(LIMoSimNodes.Get(1));

//    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
//       positionAlloc->Add (Vector (0.0, 0.0, 0.0));
//       positionAlloc->Add (Vector (5.0, 0.0, 0.0));
//    mobilityHelper.SetPositionAllocator (positionAlloc);
//    mobilityHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
//    mobilityHelper.Install(LIMoSimNodes);

    VehicleManager *vehicleManager = VehicleManager::getInstance();
    UI::Data::UIDataManager *uidm = UI::Data::UIDataManager::getInstance();
    using namespace delivery;

    vehicleManager->enableMobilityBroadcastHelper();

    mobility::car::DeliveryTruck *truck = vehicleManager->createDeliveryTruck(
                "T0",
                delivererCount,
                deliveryList
                );

//    std::string truckIpv4Address = "192.168.0.1";
//    std::string uavIpv4AddressBase = "192.168.0.";
//    uint port = 8080;
    uidm->getVehicleData(truck->getId())->getShape()->setColor("red");
    // CAUTION: The ns3 truck delivery strategy is an ns3 object
    // wrapped in a smart pointer so deletion of the reference object
    // could have unforeseen consequences.
//    auto strategy = CreateObjectWithAttributes<StrategicModels::NS3_TruckDeliveryStrategy>(
//                "TruckId", StringValue("T0")
//                );
//    Ptr<UdpTransciever> truckTransciever = CreateObjectWithAttributes<UdpTransciever>(
//                "Destination", Ipv4AddressValue(interfaces.GetAddress(1)),
//                "Port", UintegerValue(port)
//                );
//    strategy->initializeMobility("T0");
//    LIMoSimNodes.Get(0)->AddApplication(truckTransciever);
//    LIMoSimNodes.Get(0)->AggregateObject(truckTransciever);
//    LIMoSimNodes.Get(0)->AddApplication(strategy);
//    strategy->SetStartTime(Seconds(0));
//    truckTransciever->SetStartTime(Seconds(0));
//    truck->setStrategy(&*strategy);
//    truck->initialize();


    for (uint i = 0 ; i < delivererCount; i++) {
        UAV *uav = vehicleManager->createUAV("U"+ std::to_string(i));
        uav->setPosition(truck->getPosition() + Vector3d(100,i*7,100));

//        Ptr<Behaviors::NS3_Behavior_Delivery> behavior = CreateObjectWithAttributes<Behaviors::NS3_Behavior_Delivery>(
//                    "TruckId", StringValue("T0"),
//                    "VehicleId", StringValue(uav->getId())
//                    );
//        behavior->initializeMobility();
//        behavior->Setup("T0", usePrediction, uav->getId());
//        LIMoSimNodes.Get(1)->GetObject<ns3::MobilityModel>()->SetPosition(toNS3Vector(uav->getPosition()));
//        uav->setBehavior(&*behavior);

//        Ptr<UdpTransciever> uavTransciever = CreateObjectWithAttributes<UdpTransciever>(
//                    "Destination", Ipv4AddressValue(interfaces.GetAddress(0)),
//                    "Port", UintegerValue(port)
//                    );
//        LIMoSimNodes.Get(1)->AddApplication(uavTransciever);
//        LIMoSimNodes.Get(1)->AggregateObject(uavTransciever);
//        LIMoSimNodes.Get(1)->AddApplication(behavior);
//        behavior->SetStartTime(Seconds(0));

        uav->setPosition(Vector3d(80,10,60));
//        uavTransciever->SetStartTime(Seconds(0));
    }

    bool disableUl = true, disableDl = true, disablePl = false;
    Time interPacketInterval = MilliSeconds(100);
    uint16_t packetSize = 1000;
    NodeContainer ueNodes;
    ueNodes.Add(LIMoSimNodes);

    // Install and start applications on UEs and remote host
    uint16_t dlPort = 1100;
    uint16_t ulPort = 2000;
    uint16_t otherPort = 3000;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;
    for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
        // Downlink remote host -> UE
//        if (!disableDl)
//        {
//            PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
//            serverApps.Add (dlPacketSinkHelper.Install (ueNodes.Get(u)));

//            UdpClientHelper dlClient (interfaces.GetAddress (u), dlPort);
//            dlClient.SetAttribute ("Interval", TimeValue (interPacketInterval));
//            dlClient.SetAttribute ("MaxPackets", UintegerValue (1000000));
//            dlClient.SetAttribute ("PacketSize", UintegerValue(packetSize));
//            clientApps.Add (dlClient.Install (remoteHost));
//        }

//        // Uplink remote host <- UE
//        if (!disableUl)
//        {
//            ++ulPort;
//            PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
//            serverApps.Add (ulPacketSinkHelper.Install (remoteHost));

//            UdpClientHelper ulClient (remoteHostAddr, ulPort);
//            ulClient.SetAttribute ("Interval", TimeValue (interPacketInterval));
//            ulClient.SetAttribute ("MaxPackets", UintegerValue (1000000));
//            ulClient.SetAttribute ("PacketSize", UintegerValue(packetSize));
//            clientApps.Add (ulClient.Install (ueNodes.Get(u)));
//        }

        // UE Multicast UE(v) -> UE(u)
        if (!disablePl /*&& delivererCount > 1*/)
        {
            uint16_t port = otherPort;
            // install Udpserver app on node u
            PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), otherPort));
            serverApps.Add (packetSinkHelper.Install (ueNodes.Get(u)));

            // install udp client apps on each other node with node u as server
            for (uint32_t v = 0; v != u && v < ueNodes.GetN (); ++v){
//                ++port;
                UdpClientHelper client (interfaces.GetAddress (u), port);
                client.SetAttribute ("Interval", TimeValue (interPacketInterval));
                client.SetAttribute ("MaxPackets", UintegerValue (1000000));
                client.SetAttribute ("PacketSize", UintegerValue(packetSize));
                clientApps.Add (client.Install (ueNodes.Get (v)));
            }
        }
    }


    // Start Apps
    serverApps.Start (MilliSeconds (10));
    clientApps.Start (MilliSeconds (40));

    // MacTx
    Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx",
       MakeCallback (&SendPacketCallback));
    // MacRx
    Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx",
       MakeCallback (&ReceivePacketCallback));
    // Packet sink Rx
    Config::Connect ("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx",
                 MakeCallback(&ReceivePacketInSink));

    // Simulation setup end

    if (monitorPerformance()) {
        PerformanceMonitor::getInstance()->reset();
        PerformanceMonitor::getInstance()->setupWithExport("../results/Performance_" +
                                                           Simulation::getInstance()->getName() + "_" +
                                                           std::to_string(_runCount) +
                                                           ".csv");
    }

    setupIpv4TrafficAnimation();
    startSimulation();

}

} // namespace Wave
} // namespace ParcelDelivery
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim
