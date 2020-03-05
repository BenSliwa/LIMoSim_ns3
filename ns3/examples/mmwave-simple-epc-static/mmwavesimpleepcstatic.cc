#include "mmwavesimpleepcstatic.h"

#include "ns3/ns3setuphelpers.h"

#include "ns3/../helper/mmwave-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store.h"
#include "ns3/../helper/mmwave-point-to-point-epc-helper.h"


#include "ns3/callbacks.h"


#include "LIMoSim/simulation/performancemonitor.h"
#include "LIMoSim/simulation/simulation.h"

namespace LIMoSim {
namespace NS3 {
namespace Examples {
namespace MmwaveSimpleEpcStatic {

using namespace ns3;
using namespace mmwave;

inline std::string exampleName() {
    return "MmwaveSimpleEpcStatic";
}

NS_LOG_COMPONENT_DEFINE (exampleName());

void setup(int runCount, uint16_t numOfUes, uint16_t simTime_s)
{


    LogComponentEnable("MmwaveSimpleEpcStatic", LOG_LEVEL_INFO);
    NS_LOG_INFO("setting up ns3 simulation setup of " << exampleName());
    std::cout << "setting up ns3 simulation setup of " << exampleName() <<std::endl;
    Simulation::getInstance()->setName(exampleName() + "_" +
                                       "t" + std::to_string(simTime_s)+ "_" +
                                       "ue" + std::to_string(numOfUes) + "_"+
                                       "ipi"+ std::to_string(100) + "_" +
                                       "ps" + std::to_string(15))->setRunCount(runCount);
    std::cout << "Scenario params: " <<std::endl;
    std::cout << "- sim time: " << simTime_s << " seconds" << std::endl;
    std::cout << "- num of UEs: " << numOfUes << std::endl;
    std::cout << "- inter packet interval: " << 100 << " seconds"<< std::endl;
    std::cout << "- packet size: " << 15 << " Bytes" << std::endl;

    PerformanceMonitor::getInstance()->reset();
    PerformanceMonitor::getInstance()->setupWithExport("../results/Performance_" +
                                                       Simulation::getInstance()->getName() + "_" +
                                                       std::to_string(runCount) +
                                                       ".csv");

    uint16_t numEnb = 1;
    uint16_t numUe =  numOfUes;
    double simTime = simTime_s;
    uint16_t interPacketInterval = 100;  // 500 microseconds
    uint64_t packetSize = 15;
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
    bool showProgress = true;

        // Command line arguments
//        CommandLine cmd;
//        cmd.AddValue("numEnb", "Number of eNBs", numEnb);
//        cmd.AddValue("numUe", "Number of UEs per eNB", numUe);
//        cmd.AddValue("simTime", "Total duration of the simulation [s])", simTime);
//        cmd.AddValue("interPacketInterval", "Inter-packet interval [us])", interPacketInterval);
//        cmd.AddValue("harq", "Enable Hybrid ARQ", harqEnabled);
//        cmd.AddValue("rlcAm", "Enable RLC-AM", rlcAmEnabled);
//        cmd.AddValue("symPerSf", "OFDM symbols per subframe", symPerSf);
//        cmd.AddValue("sfPeriod", "Subframe period = 4.16 * symPerSf", sfPeriod);
//        cmd.AddValue("fixedTti", "Fixed TTI scheduler", fixedTti);
//        cmd.AddValue("run", "run for RNG (for generating different deterministic sequences for different drops)", fixedTti);
//        cmd.Parse(argc, argv);

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
    Config::SetDefault ("ns3::MmWaveBeamforming::SmallScaleFading", BooleanValue (smallScale));
    Config::SetDefault ("ns3::MmWaveBeamforming::FixSpeed", BooleanValue (true));
    Config::SetDefault ("ns3::MmWaveBeamforming::UeSpeed", DoubleValue (speed));

    RngSeedManager::SetSeed (1234);
    RngSeedManager::SetRun (run);

    Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper> ();
    mmwaveHelper->SetSchedulerType ("ns3::MmWaveFlexTtiMacScheduler");
    mmwaveHelper->SetAttribute ("ChannelModel", StringValue ("ns3::MmWave3gppChannel"));
    mmwaveHelper->SetAttribute ("PathlossModel", StringValue ("ns3::MmWave3gppPropagationLossModel"));
    Ptr<MmWavePointToPointEpcHelper>  epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
    mmwaveHelper->SetEpcHelper (epcHelper);
    mmwaveHelper->SetHarqEnabled (harqEnabled);

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults();

      // parse again so you can override default values from the command line
//      cmd.Parse(argc, argv);

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

    NodeContainer ueNodes;
    NodeContainer enbNodes;
    enbNodes.Create(numEnb);
    ueNodes.Create(numUe);

    // Install Mobility Model
    Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();
    enbPositionAlloc->Add (Vector (0.0, 0.0, 20.0));
    MobilityHelper enbmobility;
    enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    enbmobility.SetPositionAllocator(enbPositionAlloc);
    enbmobility.Install (enbNodes);

    MobilityHelper uemobility;
    Ptr<ListPositionAllocator> uePositionAlloc = CreateObject<ListPositionAllocator> ();
    Ptr<UniformRandomVariable> distRv = CreateObject<UniformRandomVariable> ();
    for (unsigned i = 0; i < numUe; i++)
    {
        double dist = distRv->GetValue (minDistance, maxDistance);
        uePositionAlloc->Add (Vector (dist, 0.0, 5.0));
    }
    uemobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    uemobility.SetPositionAllocator(uePositionAlloc);
    uemobility.Install (ueNodes);

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
        // Downlink remote host -> UE
//        if (!disableDl)
//        {
            PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
            serverApps.Add (dlPacketSinkHelper.Install (ueNodes.Get(u)));

            UdpClientHelper dlClient (ueIpIface.GetAddress (u), dlPort);
            dlClient.SetAttribute ("Interval", TimeValue (MilliSeconds(interPacketInterval)));
            dlClient.SetAttribute ("MaxPackets", UintegerValue (1000000));
            dlClient.SetAttribute ("PacketSize", UintegerValue(packetSize));
            clientApps.Add (dlClient.Install (remoteHost));
//        }

        // Uplink remote host <- UE
//        if (!disableUl)
//        {
            ++ulPort;
            PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
            serverApps.Add (ulPacketSinkHelper.Install (remoteHost));

            UdpClientHelper ulClient (remoteHostAddr, ulPort);
            ulClient.SetAttribute ("Interval", TimeValue (MilliSeconds(interPacketInterval)));
            ulClient.SetAttribute ("MaxPackets", UintegerValue (1000000));
            ulClient.SetAttribute ("PacketSize", UintegerValue(packetSize));
            clientApps.Add (ulClient.Install (ueNodes.Get(u)));
//        }
    }

    // Display simulator time progress
    if (showProgress)
    {
        uint32_t status_period = 1;
        Simulator::Schedule(Seconds(1),&Callbacks::PrintProgress, status_period);
    }

    serverApps.Start (MilliSeconds (10));
    clientApps.Start (MilliSeconds (40));
//    mmwaveHelper->EnableTraces ();
    // Uncomment to enable PCAP tracing
    //p2ph.EnablePcapAll("mmwave-epc-simple");

    Simulator::Stop(Seconds(simTime));
    //      Simulator::Run();

    /*GtkConfigStore config;
    config.ConfigureAttributes();*/

    //      Simulator::Destroy();

    wireUpLIMoSimAndStartSimulation();
}


} // namespace MmwaveSimpleEpcStatic
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim
