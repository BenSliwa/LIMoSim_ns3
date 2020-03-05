#include "mmwavebeamsteering.h"

// LIMoSim
#include "LIMoSim/simulation/simulation.h"
#include "LIMoSim/mobility/uav/reynolds/behaviors.h"
#include "LIMoSim/world/world.h"

// LIMoSim NS3
#include "ns3/callbacks.h"
#include "ns3/ns3setuphelpers.h"

// ns3
#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/mmwave-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"
#include "ns3/building.h"
#include "ns3/buildings-helper.h"


#include "LIMoSim/settings/filehandler.h"
#include "LIMoSim/utils/typedefs.h"
#include "LIMoSim/settings/osm/wgs84.h"
#include <QString>
#include <QStringList>

namespace LIMoSim {
namespace NS3 {
namespace Examples {
namespace MmwaveBeamSteering {

using namespace ns3;
using namespace mmwave;
using Node = ns3::Node;
using namespace LIMoSim::utils::typedefs;

NS_LOG_COMPONENT_DEFINE ("MmwaveBeamSteering");

inline std::string exampleName() {
    return "MmwaveBeamSteering";
}

std::vector<Vector3d> readBaseStationPositions() {
    StringVector basestations = LIMoSim::FileHandler::read("./basestations-telekom.txt");
    basestations.erase(basestations.begin());
    basestations.erase(basestations.end());
    Vector3d origin = World::getInstance()->getOrigin();
    std::vector<Vector3d> bsposs;
    for (std::string s : basestations) {
        std::cout << s /*<< std::endl*/;
        QStringList parts = QString::fromStdString(s).split(';');
        if(parts.size()  != 3) continue;
        Vector3d pos(parts.at(1).toDouble(), parts.at(2).toDouble());
        Vector3d cts = WGS84::computeOffset(pos, origin);
        std::cout << cts << std::endl;
        bsposs.push_back(Vector3d(cts.x, cts.y));
    }
    return bsposs;
}
Behavior* behaviorFactory() {
//    return new Behavior_CircleAround(Vector3d(0, 0, 0), 50.0);
//    return new Behavior_SeekWithSpeed(Vector3d(-60, 60), 40);
//    return new Behavior_Arrive(Vector3d(-60, 60), 10);
    return new Behavior_NoOp();
}

void setup()
{
    LogComponentEnable("MmWaveHelper", LOG_LEVEL_ALL);
    LogComponentEnable("MmwaveBeamSteering", LOG_LEVEL_INFO);
    NS_LOG_INFO("setting up ns3 simulation setup of " << "MmwaveBeamSteering");
    std::cout << "setting up ns3 simulation setup of " << "MmwaveBeamSteering" <<std::endl;
    Simulation::getInstance()->setName(exampleName())->setRunCount(0);
    std::vector<Vector3d> enbpos = readBaseStationPositions();
    uint16_t numEnb = enbpos.size();
    uint16_t numUe = 1;
    double simTime = 625.0;
    double interPacketInterval = 5;  // 500 microseconds
    uint64_t packetSize = 1150;
    double minDistance = 10.0;  // eNB-UE distance in meters
    double maxDistance = 150.0;  // eNB-UE distance in meters
    bool harqEnabled = true;
    bool rlcAmEnabled = false;
    bool fixedTti = false;
    unsigned symPerSf = 24;
    double sfPeriod = 100.0;
    unsigned run = 0;
    bool smallScale = true;
    double speed = 3;

    Config::SetDefault ("ns3::MmWaveHelper::RlcAmEnabled", BooleanValue(rlcAmEnabled));
    Config::SetDefault ("ns3::MmWaveHelper::HarqEnabled", BooleanValue(harqEnabled));
    Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue(harqEnabled));
    Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::CqiTimerThreshold", UintegerValue(1000));
    Config::SetDefault ("ns3::MmWaveFlexTtiMaxWeightMacScheduler::HarqEnabled", BooleanValue(harqEnabled));
    Config::SetDefault ("ns3::MmWaveFlexTtiMaxWeightMacScheduler::FixedTti", BooleanValue(fixedTti));
    Config::SetDefault ("ns3::MmWaveFlexTtiMaxWeightMacScheduler::SymPerSlot", UintegerValue(6));
    Config::SetDefault ("ns3::MmWavePhyMacCommon::ResourceBlockNum", UintegerValue(1));
    Config::SetDefault ("ns3::MmWavePhyMacCommon::ChunkPerRB", UintegerValue(72));
    Config::SetDefault ("ns3::MmWavePhyMacCommon::SymbolsPerSubframe", UintegerValue(symPerSf));
    Config::SetDefault ("ns3::MmWavePhyMacCommon::SubframePeriod", DoubleValue(sfPeriod));
    Config::SetDefault ("ns3::MmWavePhyMacCommon::TbDecodeLatency", UintegerValue(200.0));
    Config::SetDefault ("ns3::MmWaveBeamforming::LongTermUpdatePeriod", TimeValue (MilliSeconds (100.0)));
    Config::SetDefault ("ns3::LteEnbRrc::SystemInformationPeriodicity", TimeValue (MilliSeconds (5.0)));
    //Config::SetDefault ("ns3::MmWavePropagationLossModel::ChannelStates", StringValue ("n"));
    Config::SetDefault ("ns3::LteRlcAm::ReportBufferStatusTimer", TimeValue(MicroSeconds(100.0)));
    Config::SetDefault ("ns3::LteRlcUmLowLat::ReportBufferStatusTimer", TimeValue(MicroSeconds(100.0)));
    Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue (320));
    Config::SetDefault ("ns3::LteEnbRrc::FirstSibTime", UintegerValue (2));
//    Config::SetDefault ("ns3::MmWaveBeamforming::SmallScaleFading", BooleanValue (smallScale));
//    Config::SetDefault ("ns3::MmWaveBeamforming::FixSpeed", BooleanValue (false));
//    Config::SetDefault ("ns3::MmWaveBeamforming::UeSpeed", DoubleValue (speed));
    Config::SetDefault ("ns3::AntennaArrayModel::IsotropicAntennaElements", BooleanValue(false));
//    Config::SetDefault ("ns3::MmWave3gppChannel::DirectBeam", BooleanValue(false));
//    Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::Scenario", StringValue("UMi-StreetCanyon"));
//    Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::ChannelCondition", StringValue("l"));

    RngSeedManager::SetSeed (1234);
    RngSeedManager::SetRun (run);

    Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper> ();
//    mmwaveHelper->SetAttribute ("PathlossModel", StringValue ("ns3::BuildingsObstaclePropagationLossModel"));
    mmwaveHelper->Initialize ();
    mmwaveHelper->SetSchedulerType ("ns3::MmWaveFlexTtiMacScheduler");
    mmwaveHelper->SetHarqEnabled (harqEnabled);
//    mmwaveHelper->SetAttribute ("ChannelModel", StringValue ("ns3::MmWave3gppChannel"));
//    mmwaveHelper->SetAttribute ("PathlossModel", StringValue ("ns3::MmWave3gppPropagationLossModel"));
    mmwaveHelper->SetAttribute ("LtePathlossModel", StringValue ("ns3::DeterministicObstacleShadowingSpectrumPropagationLossModel"));
    Ptr<MmWavePointToPointEpcHelper>  epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
    mmwaveHelper->SetEpcHelper (epcHelper);

    Ptr<Node> pgw = epcHelper->GetPgwNode ();

    // Create a single RemoteHost
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create (1);
    Ptr<Node> remoteHost = remoteHostContainer.Get (0);
    InternetStackHelper internet;
    internet.Install (remoteHostContainer);

    // Create the Internet
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
    p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
    p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
    NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
    // interface 0 is localhost, 1 is the p2p device
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
    remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);


    // Add LIMoSim buildings to ns3
//    World * world = World::getInstance();
//    auto buildings = world->getBuildings();
//    for (auto building : buildings) {
//        Ptr<ns3::Building> ns3Building  = Create<ns3::Building>();
//        Vector3d start = building.second->getStartNode()->getPosition();
//        Vector3d end = building.second->getEndNode()->getPosition();
//        ns3Building->SetBoundaries(Box(start.x, start.x + 60, start.y, start.y + 40, 0, 50));
//    }


    NodeContainer ueNodes;
    NodeContainer enbNodes;

    enbNodes.Create(numEnb);
    ueNodes.Create(numUe);

    // Setup LIMoSim mobility
    std::vector<VehicleProperties> vehiclesProps = {
        VehicleProperties(
            Vector(40, 60, 5),
            LIMoSimUAV,
            "U1",
            VehicleUIProps("yellow"),
            MakeCallback(&behaviorFactory)
                )
    };
    MobilityHelper mobility;
    setupNS3MobilityFromVehicleProps(mobility, vehiclesProps, ueNodes);

    // Install ns3 Mobility Model
    Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();
    for (uint i = 0; i < numEnb; i++) {
        enbPositionAlloc->Add (Vector (enbpos.at(i).x, enbpos.at(i).y, 5.0));
    }
//    enbPositionAlloc->Add (Vector (0.0, 0.0, 0.0));
    MobilityHelper enbmobility;
    enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    enbmobility.SetPositionAllocator(enbPositionAlloc);
    enbmobility.Install (enbNodes);

    BuildingsHelper::Install(enbNodes);
    BuildingsHelper::Install(ueNodes);

    // Install mmWave Devices to the nodes
    NetDeviceContainer enbmmWaveDevs = mmwaveHelper->InstallEnbDevice (enbNodes);
    NetDeviceContainer uemmWaveDevs = mmwaveHelper->InstallUeDevice (ueNodes);

    // Install the IP stack on the UEs
    internet.Install (ueNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (uemmWaveDevs));
    // Assign IP address to UEs, and install applications
    for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

    mmwaveHelper->AttachToClosestEnb (uemmWaveDevs, enbmmWaveDevs);

    // Install and start applications on UEs and remote host
    uint16_t dlPort = 1234;
    uint16_t ulPort = 2000;
    uint16_t otherPort = 3000;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;
    for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
        ++ulPort;
        ++otherPort;
        PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
        PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
        PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), otherPort));
        serverApps.Add (dlPacketSinkHelper.Install (ueNodes.Get (u)));
        serverApps.Add (ulPacketSinkHelper.Install (remoteHost));
        serverApps.Add (packetSinkHelper.Install (ueNodes.Get (u)));

        UdpClientHelper dlClient (ueIpIface.GetAddress (u), dlPort);
        dlClient.SetAttribute ("Interval", TimeValue (MilliSeconds (interPacketInterval)));
        dlClient.SetAttribute ("MaxPackets", UintegerValue (1000000));
        dlClient.SetAttribute ("PacketSize", UintegerValue(packetSize));

        UdpClientHelper ulClient (remoteHostAddr, ulPort);
        ulClient.SetAttribute ("Interval", TimeValue (MilliSeconds (interPacketInterval)));
        ulClient.SetAttribute ("MaxPackets", UintegerValue (1000000));
        ulClient.SetAttribute ("PacketSize", UintegerValue(packetSize));

    //      UdpClientHelper client (ueIpIface.GetAddress (u), otherPort);
    //      client.SetAttribute ("Interval", TimeValue (MilliSeconds(interPacketInterval)));
    //      client.SetAttribute ("MaxPackets", UintegerValue(1000000));

        clientApps.Add (dlClient.Install (remoteHost));
        clientApps.Add (ulClient.Install (ueNodes.Get(u)));
    //      if (u+1 < ueNodes.GetN ())
    //        {
    //          clientApps.Add (client.Install (ueNodes.Get(u+1)));
    //        }
    //      else
    //        {
    //          clientApps.Add (client.Install (ueNodes.Get(0)));
    //        }
    }

    uint32_t status_period = 1;
    Simulator::Schedule(Seconds(1),&Callbacks::PrintProgress, status_period);

    BuildingsHelper::MakeMobilityModelConsistent();
    mmwaveHelper->EnableTraces ();
    // Uncomment to enable PCAP tracing
    p2ph.EnablePcapAll("mmwave-epc-simple");

    Simulator::Stop(Seconds(simTime));

//    LIMoSim::Simulation::getInstance()->run();
    wireUpLIMoSimAndStartSimulation();
}



} // namespace MmwaveBeamSteering
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim
