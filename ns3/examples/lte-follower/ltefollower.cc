#include "ltefollower.h"

// ns3
#include "ns3/core-module.h"
#include "ns3/lte-helper.h"
#include "ns3/lte-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include <ns3/buildings-helper.h>
#include <ns3/antenna-module.h>
#include <ns3/point-to-point-helper.h>
#include <ns3/config-store.h>

// LIMoSim ns3
#include "ns3/callbacks.h"
#include "ns3/ns3setuphelpers.h"
#include "ns3/limosimmobilitymodel.h"
#include "ns3/applications/mobilityreceiverudp.h"
#include "ns3/applications/mobilitysenderudp.h"

// LIMoSim
#include "LIMoSim/mobility/uav/reynolds/behaviors.h"
#include "LIMoSim/simulation/performancemonitor.h"
#include "LIMoSim/world/vehiclemanager.h"

namespace LIMoSim {
namespace NS3 {
namespace Examples {
namespace LteFollower {

using namespace ns3;
using Node = ns3::Node;

inline std::string exampleName() {
    return "LteFollower";
}

NS_LOG_COMPONENT_DEFINE (exampleName());

Behavior* behaviorFactory(std::string _targetId) {
//    return new Behavior_FollowAtElevation(_targetId);
    return new Behavior_FollowAtElevation(_targetId);
}

std::vector<VehicleProperties> generateCarProps(uint numberOfUes, std::vector<std::string> colors) {
    std::vector<VehicleProperties> vehiclesProps;
    for (uint i = 0; i < numberOfUes; i++) {
        vehiclesProps.push_back(
                    VehicleProperties(
                        Vector(0,0,0),
                        LIMoSimCar,
                        "C" + std::to_string(i),
                        VehicleUIProps(colors.at(i%colors.size()))
                        )
                    );
    }
    return vehiclesProps;
}

std::vector<VehicleProperties> generateUAVProps(uint numberOfUAVs, std::vector<std::string> colors) {
    std::vector<VehicleProperties> UAVProps;
    for (uint i = 0; i < numberOfUAVs; i++) {
        UAVProps.push_back(
                    VehicleProperties(
                        Vector(i*50 + 400,300,40),
                        LIMoSimUAV,
                        "U" + std::to_string(i),
                        VehicleUIProps(colors.at(i%colors.size())),
                        MakeBoundCallback(&behaviorFactory, "C" + std::to_string(i))
                        )
                    );
    }
    return UAVProps;
}

void setup(int runCount,
           uint16_t numOfPairs,
           uint16_t simTime_s,
           uint16_t interPacketInterval_ms,
           uint16_t packetSize_B)
{
    LogComponentEnable(exampleName().c_str(), LOG_LEVEL_INFO);
    NS_LOG_INFO("setting up ns3 simulation setup of " << exampleName());
    std::cout << "setting up ns3 simulation setup of " << exampleName() << " run count:" << runCount <<std::endl;
    Simulation::getInstance()->setName(exampleName() + "_" +
                                       "t" + std::to_string(simTime_s)+ "_" +
                                       "np" + std::to_string(numOfPairs) + "_"+
                                       "ipi"+ std::to_string(interPacketInterval_ms) + "_" +
                                       "ps" + std::to_string(packetSize_B))->setRunCount(runCount);
    std::cout << "Scenario params: " <<std::endl;
    std::cout << "- sim time: " << simTime_s << " seconds" << std::endl;
    std::cout << "- num of Pairs: " << numOfPairs << std::endl;
    std::cout << "- inter packet interval: " << interPacketInterval_ms << " seconds"<< std::endl;
    std::cout << "- packet size: " << packetSize_B << " Bytes" << std::endl;

    PerformanceMonitor::getInstance()->reset();
    PerformanceMonitor::getInstance()->setupWithExport("../results/Performance_" +
                                                       Simulation::getInstance()->getName() + "_" +
                                                       std::to_string(runCount) +
                                                       ".csv");

    setUAVNextToCar(true);

    uint16_t numberOfEnbs = 1;
    uint16_t numberOfUes = numOfPairs * 2;
    Time simTime = Seconds (simTime_s);
    Time interPacketInterval = MilliSeconds (interPacketInterval_ms);
    Time burstPeriod = Seconds(10);
    uint64_t packetSize = packetSize_B; // in Bytes - ns3 max is 63KB
    uint64_t burstSize = 1024;

    bool disableDl = true;
    bool disableUl = true;
    bool disablePl = false;
    bool buildings = true;
    bool showProgress = true;
    bool disablePeriodicalBurst = true;


    double ueTxPower = 23.0;
    double eNBTxPower = 43.0;
    uint32_t dlEarfcn = 100;
    uint32_t ulEarfcn = 18100;


    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults ();

    srand(static_cast<unsigned int>(time(nullptr)));
    uint seed = static_cast<uint>(rand());
    RngSeedManager::SetSeed (seed);
    RngSeedManager::SetRun (static_cast<uint>(runCount));

    Config::SetDefault ("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue (false));
    Config::SetDefault ("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue (false));
    Config::SetDefault ("ns3::LteAmc::AmcModel", EnumValue (LteAmc::PiroEW2010));
    Config::SetDefault ("ns3::LteAmc::Ber", DoubleValue (0.00005));
    Config::SetDefault ("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue(100));
    Config::SetDefault ("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue(100));
    Config::SetDefault ("ns3::LteEnbNetDevice::UlEarfcn", UintegerValue(dlEarfcn));
    Config::SetDefault ("ns3::LteEnbNetDevice::DlEarfcn", UintegerValue(ulEarfcn));
    Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (eNBTxPower));
    Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue (ueTxPower));

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
    if (buildings) {
        lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::HybridBuildingsPropagationLossModel"));
        lteHelper->SetPathlossModelAttribute ("ShadowSigmaOutdoor", DoubleValue (0.0));
        lteHelper->SetPathlossModelAttribute ("ShadowSigmaIndoor", DoubleValue (0.0));
        lteHelper->SetPathlossModelAttribute ("ShadowSigmaExtWalls", DoubleValue (0.0));
    } else {
        lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::FriisSpectrumPropagationLossModel"));
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



    NodeContainer enbNodes;
    NodeContainer carNodes;
    NodeContainer uavNodes;
    enbNodes.Create(numberOfEnbs);
    uavNodes.Create(numOfPairs);
    carNodes.Create(numOfPairs);
    NodeContainer ueNodes (uavNodes, carNodes);

    int utilcount = 5;
    std::vector<std::string> colors { "red", "blue", "yellow",
                                      "magenta", "cyan"};

    std::vector<VehicleProperties> vehiclesProps = generateCarProps(numOfPairs, colors);
    std::vector<VehicleProperties> UAVProps = generateUAVProps(numOfPairs, colors);
    vehiclesProps.insert(vehiclesProps.begin(), UAVProps.begin(), UAVProps.end());

    MobilityHelper mobility;
    setupNS3MobilityFromVehicleProps(mobility, vehiclesProps, ueNodes);

    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
    for (uint16_t i = 0; i < numberOfEnbs; i++)
    {
        positionAlloc->Add (Vector(i*200, i*200, 50)); // ENBs positions
    }

    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(enbNodes);

    if (buildings) {
        BuildingsHelper::Install (enbNodes);
        BuildingsHelper::Install (ueNodes);
        BuildingsHelper::MakeMobilityModelConsistent();
    }

    // Install LTE Devices to the nodes
    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);

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

    // Assign each UEs to one eNodeB
//    for (uint16_t i = 0; i < numberOfUes; i++)
//    {
        lteHelper->Attach (ueLteDevs, enbLteDevs.Get(0));
//    }

    // Install and start applications on UEs and remote host
    uint16_t dlPort = 1100;
    uint16_t ulPort = 2000;
    uint16_t dlPort1 = 1101;
    uint16_t ulPort1 = 2001;
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
            clientApps.Add (ulClient.Install (ueNodes.Get(u)));



            if (!disablePeriodicalBurst) {
                ++ulPort;
                PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
                serverApps.Add (ulPacketSinkHelper.Install (remoteHost));

                UdpClientHelper ulClient (remoteHostAddr, ulPort);
                ulClient.SetAttribute ("Interval", TimeValue (burstPeriod));
                ulClient.SetAttribute ("MaxPackets", UintegerValue (1000000));
                ulClient.SetAttribute ("PacketSize", UintegerValue(burstSize));
                clientApps.Add (ulClient.Install (ueNodes.Get(u)));
            }
        }

        // UE(uav) -> UE(car)
        if (!disablePl)
        {

            uint32_t receivingNodeIdx = 0;
            uint16_t port = 0;
            if (u < numOfPairs) {
                // current Node is UAV
                receivingNodeIdx = u + numOfPairs;
                port = dlPort1;
            } else {
                // current Node is Car
                continue;
//                receivingNodeIdx = u- numOfPairs;
//                port = ulPort1;
            }

            // install Udpserver app on node receiving node
            PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), port));
            serverApps.Add (packetSinkHelper.Install (ueNodes.Get(receivingNodeIdx)));

            // install udp client app on corresponding sending node
            UdpClientHelper client (ueIpIface.GetAddress (receivingNodeIdx), port);
            client.SetAttribute ("Interval", TimeValue (interPacketInterval));
            client.SetAttribute ("MaxPackets", UintegerValue (1000000));
            client.SetAttribute ("PacketSize", UintegerValue(packetSize));
            clientApps.Add (client.Install (ueNodes.Get (u)));
        }
    }

    serverApps.Start (MilliSeconds (10));
    clientApps.Start (MilliSeconds (40));

    lteHelper->EnablePdcpTraces();
    lteHelper->EnablePhyTraces();

    Ptr<RadioBearerStatsCalculator> pdcpStats = lteHelper->GetPdcpStats ();
    pdcpStats->SetDlPdcpOutputFilename("../results/DlPdcpStats_" +
                                       Simulation::getInstance()->getName() + "_" +
                                       std::to_string(runCount) + ".txt");
    pdcpStats->SetUlPdcpOutputFilename("../results/UlPdcpStats_" +
                                       Simulation::getInstance()->getName() + "_" +
                                       std::to_string(runCount) + ".txt");
//    Config::SetDefault ("ns3::PhyStatsCalculator::DlRsrpSinrFilename",
//                        StringValue("../results/DlRsrpSinrStats_" +
//                                    Simulation::getInstance()->getName() + "_" +
//                                    std::to_string(runCount) + ".txt"));

    // Display simulator time progress
    if (showProgress)
    {
        uint32_t status_period = 1;
        Simulator::Schedule(Seconds(1),&Callbacks::PrintProgress, status_period);
    }


    VehicleManager::getInstance()->enableMobilityBroadcastHelper();

    Simulator::Stop (simTime);
    wireUpLIMoSimAndStartSimulation();

}

} // namespace LteFollower
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim
