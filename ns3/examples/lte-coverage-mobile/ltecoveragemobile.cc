#include "ltecoveragemobile.h"


#include<ctime>

#include "ns3/core-module.h"
#include "ns3/lte-helper.h"
#include "ns3/lte-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include <ns3/buildings-helper.h>
#include <ns3/point-to-point-helper.h>
#include <ns3/ipv4.h>
#include <ns3/config-store.h>

// LIMoSim ns3
#include "ns3/callbacks.h"
#include "ns3/ns3setuphelpers.h"
#include "ns3/limosimmobilitymodel.h"
#include "ns3/applications/mobilityreceiverudp.h"
#include "ns3/applications/mobilitysenderudp.h"

// LIMoSim
#include "LIMoSim/mobility/uav/reynolds/behaviors.h"
#include "LIMoSim/mobility/car/car.h"
#include "LIMoSim/world/world.h"
#include "LIMoSim/world/road/road.h"
#include "LIMoSim/world/road/lanesegment.h"
//#include "LIMoSim/world/road/lane.h"
#include "LIMoSim/world/road/roadsegment.h"
#include "LIMoSim/simulation/performancemonitor.h"


#include "standalone/qclidecorators.h"

#include <QCommandLineParser>


namespace LIMoSim {
namespace NS3 {
namespace Examples {
namespace LteCoverageMobile {

using namespace ns3;
using Node = ns3::Node;

inline std::string exampleName() {
    return "LteCoverageMobile";
}

NS_LOG_COMPONENT_DEFINE (exampleName());

Behavior* behaviorFactory() {
    return new Behavior_Cohesion2D(1500);
//    return new Behavior_NoOp();
}

std::vector<VehicleProperties> generateCarProps(uint numberOfUes) {
    std::vector<VehicleProperties> vehiclesProps;
    for (uint i = 0; i < numberOfUes; i++) {
        vehiclesProps.push_back(
                    VehicleProperties(
                        Vector(0,0,0),
                        LIMoSimCar,
                        "C" + std::to_string(i),
                        VehicleUIProps("red")
                        )
                    );
    }
    return vehiclesProps;
}

//    Config::SetDefault ("ns3::LteEnbNetDevice::DlEarfcn", UintegerValue (100));
//    Config::SetDefault ("ns3::LteEnbNetDevice::UlEarfcn", UintegerValue (100 + 18000));
//    Config::SetDefault ("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue (dlBandwidth));
//    Config::SetDefault ("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue (ulBandwidth));
//    Config::SetDefault ("ns3::LteUeNetDevice::DlEarfcn", UintegerValue (100));

void setup(
        int _runCount,
        uint16_t _simTime_s,
        uint16_t _numOfUes,
        uint16_t _interPacketInterval_ms,
        uint16_t _packetSize_B
        )
{

    LogComponentEnable(exampleName().c_str(), LOG_LEVEL_INFO);
    NS_LOG_INFO("setting up ns3 simulation setup of " << exampleName());
    std::cout << "setting up ns3 simulation setup of " << exampleName() << " run count:" << _runCount <<std::endl;

    QCommandLineParser parser;
    parser.setApplicationDescription("LIMoSim - ns3 - lteCoverageMobile");
    parser.addHelpOption();
    parser.addVersionOption();

    Standalone::qclidecorators::addGeneralOptions(parser);
    Standalone::qclidecorators::addSimulationOptions(parser);

    QCommandLineOption simTimeOption(
                QStringList() << "t" << "sim-time",
                "Simulation time in seconds",
                "number");
    QCommandLineOption ueCountOption(
                QStringList() << "ue" << "ue-count",
                "Number of user equipments",
                "number");
    QCommandLineOption interPackterIntervalOption(
                "ipi",
                "Interarrival time of packets in milliseconds",
                "number");
    QCommandLineOption packetSizeOption(
                QStringList() << "ps" << "packet-size",
                "Size of the data packets in bytes",
                "number");
//    QCommandLineOption droneInteractionModeOption(
//                "drone-interaction",
//                "Drone interaction mode: ONSITE(0); ENROUTE(1)",
//                "number");
    QCommandLineOption runCountOption(
                QStringList() << "r" << "run-count",
                "Run count",
                "number");

    parser.addOptions({
                          simTimeOption,
                          ueCountOption,
                          interPackterIntervalOption,
                          packetSizeOption,
//                          droneInteractionModeOption,
                          runCountOption
                      });
    parser.parse(QCoreApplication::arguments());

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
    uint16_t simTime_s = parser.isSet(simTimeOption) ?
                parser.value(simTimeOption).toInt(&ok) :
                _simTime_s;
    if (!ok) {
        std::cout<<"NS3::Examples::LteCoverageMobile::setup: setting simTime fallback value: " << _simTime_s <<std::endl;
        simTime_s = _simTime_s;
        ok = true;
    }
    uint16_t numOfUes = parser.isSet(ueCountOption) ?
                parser.value(ueCountOption).toInt(&ok) :
                _numOfUes;
    if (!ok) {
        std::cout<<"NS3::Examples::LteCoverageMobile::setup: setting ue count fallback value: " << _numOfUes <<std::endl;
        numOfUes = _numOfUes;
        ok = true;
    }
    uint16_t interPacketInterval_ms = parser.isSet(interPackterIntervalOption) ?
                parser.value(interPackterIntervalOption).toInt(&ok) :
                _interPacketInterval_ms;
    if (!ok) {
        std::cout<<"NS3::Examples::LteCoverageMobile::setup: setting inter packet arrival fallback value: " << _interPacketInterval_ms <<std::endl;
        interPacketInterval_ms = _interPacketInterval_ms;
        ok = true;
    }
    uint16_t packetSize_B = parser.isSet(packetSizeOption) ?
                parser.value(packetSizeOption).toInt(&ok) :
                _packetSize_B;
    if (!ok) {
        std::cout<<"NS3::Examples::LteCoverageMobile::setup: setting packet size fallback value: " << _packetSize_B <<std::endl;
        packetSize_B = _packetSize_B;
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



    Simulation::getInstance()->setName(exampleName() + "_" +
                                       "t" + std::to_string(simTime_s)+ "_" +
                                       "ue" + std::to_string(numOfUes) + "_"+
                                       "ipi"+ std::to_string(interPacketInterval_ms) + "_" +
                                       "ps" + std::to_string(packetSize_B))->setRunCount(runCount);
    std::cout << "Scenario params: " <<std::endl;
    std::cout << "- sim time: " << simTime_s << " seconds" << std::endl;
    std::cout << "- num of UEs: " << numOfUes << std::endl;
    std::cout << "- inter packet interval: " << interPacketInterval_ms << " seconds"<< std::endl;
    std::cout << "- packet size: " << packetSize_B << " Bytes" << std::endl;

    PerformanceMonitor::getInstance()->reset();
    PerformanceMonitor::getInstance()->setupWithExport("../results/Performance_" +
                                                       Simulation::getInstance()->getName() + "_" +
                                                       std::to_string(runCount) +
                                                       ".csv");

    uint16_t numOfEnbs = 1;
    Time simTime = Seconds (simTime_s);
    Time interPacketInterval = MilliSeconds(interPacketInterval_ms);
//    uint64_t packetSize = packetSize_B; // in Bytes - ns3 max is 63KB

    bool disableDl = false;
    bool disableUl = false;
    bool disablePl = true;
    bool showProgress = true;
    bool buildings = true;

    uint32_t dlEarfcn = 100;
    uint32_t ulEarfcn = 18100;
    uint32_t dlResBlck = 100;
    uint32_t ulResBlck = 100;

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults ();

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

    srand(static_cast<unsigned int>(time(nullptr)));
    uint seed = static_cast<uint>(rand());
    RngSeedManager::SetSeed (seed);
    RngSeedManager::SetRun (static_cast<uint>(runCount));

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
    if (buildings) {
        lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::HybridBuildingsPropagationLossModel"));
        lteHelper->SetPathlossModelAttribute ("ShadowSigmaOutdoor", DoubleValue (0.0));
        lteHelper->SetPathlossModelAttribute ("ShadowSigmaIndoor", DoubleValue (0.0));
        lteHelper->SetPathlossModelAttribute ("ShadowSigmaExtWalls", DoubleValue (0.0));
    } else {
        lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::DeterministicObstacleShadowingSpectrumPropagationLossModel"));
//        lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::FriisSpectrumPropagationLossModel"));
    }
    Config::SetDefault ("ns3::LteUePhy::EnableUplinkPowerControl", BooleanValue (false));
    lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler");
    lteHelper->SetSchedulerAttribute ("UlCqiFilter", EnumValue (FfMacScheduler::PUSCH_UL_CQI));

    Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
    lteHelper->SetEpcHelper (epcHelper);

    Ptr<Node> pgw = epcHelper->GetPgwNode ();

    // Create a single RemoteHost
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create (1);
    Ptr<Node> remoteHost = remoteHostContainer.Get (0);
    InternetStackHelper internet;
    internet.Install (remoteHostContainer);

    // Create the Internet
    // IP Stack for remote host
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


    NodeContainer ueNodes;
    NodeContainer enbNodes;
    enbNodes.Create(numOfEnbs);
    ueNodes.Create(numOfUes);

    // Setup Mobility
    std::vector<VehicleProperties> vehiclesProps = generateCarProps(numOfUes);
    vehiclesProps.insert(vehiclesProps.begin(),
                         VehicleProperties(
                 //            Vector (300, 300, 150),
                             Vector (0, 0, 150),
                             LIMoSimUAV,
                             "U0",
                             VehicleUIProps("yellow"),
                             MakeCallback(&behaviorFactory)
                             )
                         );

    MobilityHelper mobility;
    NodeContainer vehicles (enbNodes, ueNodes);
    setupNS3MobilityFromVehicleProps(mobility, vehiclesProps, vehicles);

    if (buildings) {
        BuildingsHelper::Install (enbNodes);
        BuildingsHelper::Install (ueNodes);
        BuildingsHelper::MakeMobilityModelConsistent();
    }

    // Install LTE Devices to the nodes
    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);

    // Assign Ip Address to the EnB(UAV)
    Ptr<Ipv4> ipv4 = enbNodes.Get(0)->GetObject<Ipv4>();
    int32_t interface = ipv4->GetInterfaceForDevice (enbLteDevs.Get(0));
    if (interface == -1) {
        interface = static_cast<int32_t>(ipv4->AddInterface (enbLteDevs.Get(0)));
    }
    uint32_t interface_casted = static_cast<uint32_t>(interface);
    Ipv4InterfaceAddress ipv4Addr = Ipv4InterfaceAddress (Ipv4Address("7.0.0.1"), Ipv4Mask ("/16"));
    ipv4->AddAddress (interface_casted, ipv4Addr);
    ipv4->SetMetric (interface_casted, 1);
    ipv4->SetUp (interface_casted);

    // Install the IP stack on the UEs
    internet.Install (ueNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevs));
    // Assign IP address to UEs, and install applications
    for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
        Ptr<Node> ueNode = ueNodes.Get (u);
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
        ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

    // Attach all UEs to first eNodeB
    for (uint16_t i = 0; i < numOfUes; i++)
    {
        lteHelper->Attach (ueLteDevs.Get(i), enbLteDevs.Get(0));
    }

    // Install and start applications on UEs and remote host
    uint16_t dlPort = 1100;
    uint16_t ulPort = 2000;
    uint16_t otherPort = 3000;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;
    for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
        // Downlink remote host -> UE
        if (!disableDl)
        {
            PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
            serverApps.Add (dlPacketSinkHelper.Install (ueNodes.Get(u)));

            UdpClientHelper dlClient (ueIpIface.GetAddress (u), dlPort);
            dlClient.SetAttribute ("Interval", TimeValue (interPacketInterval));
            dlClient.SetAttribute ("MaxPackets", UintegerValue (1000000));
            dlClient.SetAttribute ("PacketSize", UintegerValue(packetSize_B));
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
            ulClient.SetAttribute ("PacketSize", UintegerValue(packetSize_B));
            clientApps.Add (ulClient.Install (ueNodes.Get(u)));
        }

        // UE Multicast UE(v) -> UE(u)
        if (!disablePl && numOfUes > 1)
        {
            uint16_t port = otherPort;
            // install Udpserver app on node u
            PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), otherPort));
            serverApps.Add (packetSinkHelper.Install (ueNodes.Get(u)));

            // install udp client apps on each other node with node u as server
            for (uint32_t v = 0; v != u && v < ueNodes.GetN (); ++v){
                ++port;
                UdpClientHelper client (ueIpIface.GetAddress (u), port);
                client.SetAttribute ("Interval", TimeValue (interPacketInterval));
                client.SetAttribute ("MaxPackets", UintegerValue (1000000));
                client.SetAttribute ("PacketSize", UintegerValue(packetSize_B));
                clientApps.Add (client.Install (ueNodes.Get (v)));
            }
        }
    }

    // Install applications for mobility data transmission
    ApplicationContainer mobilitySenderApps;
    ApplicationContainer mobilityReceiverApps;
    for (uint8_t n = 0; n  < ueNodes.GetN(); n++) {
        Ptr<ns3::Node> node = ueNodes.Get(n);
        Ptr<Applications::MobilitySenderUDP> senderApp = CreateObject<Applications::MobilitySenderUDP>();
        node->AddApplication (senderApp);
    }
    for(uint8_t n = 0; n < enbNodes.GetN(); n++) {
        Ptr<ns3::Node> node = enbNodes.Get (n);
        Ptr<Applications::MobilityReceiverUDP> receiverApp = CreateObject<Applications::MobilityReceiverUDP>();
        node->AddApplication (receiverApp);
    }
    Config::Set ("/NodeList/*/ApplicationList/*/$MobilitySenderUDP/Destination",
                 Ipv4AddressValue ("7.0.0.1"));



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

    // Display simulator time progress
    if (showProgress)
    {
        uint32_t status_period = 1;
        Simulator::Schedule(Seconds(1),&Callbacks::PrintProgress, status_period);
    }

    serverApps.Start (MilliSeconds (10));
    clientApps.Start (MilliSeconds (40));
    mobilitySenderApps.Start (MilliSeconds (40));
    mobilityReceiverApps.Start (MilliSeconds (10));

    lteHelper->EnablePdcpTraces();
    lteHelper->EnablePhyTraces();

    Ptr<RadioBearerStatsCalculator> pdcpStats = lteHelper->GetPdcpStats ();
    pdcpStats->SetDlPdcpOutputFilename("../results/DlPdcpStats_" +
                                       Simulation::getInstance()->getName() + "_" +
                                       std::to_string(runCount) + ".txt");
    pdcpStats->SetUlPdcpOutputFilename("../results/UlPdcpStats_" +
                                       Simulation::getInstance()->getName() + "_" +
                                       std::to_string(runCount) + ".txt");

    Simulator::Stop (simTime);
    wireUpLIMoSimAndStartSimulation();

}

} // namespace LteCoverageMobile
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim
