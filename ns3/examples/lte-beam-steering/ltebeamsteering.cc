#include "ltebeamsteering.h"

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

// LIMoSim ns3
#include "ns3/ns3setuphelpers.h"
#include "ns3/limosimmobilitymodel.h"
#include "ns3/applications/mobilityreceiverudp.h"
#include "ns3/applications/mobilitysenderudp.h"

// LIMoSim
#include "LIMoSim/mobility/uav/reynolds/behaviors.h"

namespace LIMoSim {
namespace NS3 {
namespace Examples {
namespace LteBeamSteering {

inline std::string exampleName() {
    return "LteBeamSteering";
}

NS_LOG_COMPONENT_DEFINE (exampleName());

Behavior* behaviorFactory(std::string _targetId) {
//    return new Behavior_FollowAtElevation(_targetId);
    return new Behavior_FollowAtElevation(_targetId);
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

std::vector<VehicleProperties> generateUAVProps(uint numberOfUAVs) {
    std::vector<VehicleProperties> UAVProps;
    for (uint i = 0; i < numberOfUAVs; i++) {
        UAVProps.push_back(
                    VehicleProperties(
                        Vector(i*50 + 400,300,60),
                        LIMoSimUAV,
                        "U" + std::to_string(i),
                        VehicleUIProps("red"),
                        MakeBoundCallback(&behaviorFactory, "C" + std::to_string(i))
                        )
                    );
    }
    return UAVProps;
}


void setup(int runCount)
{
    LogComponentEnable(exampleName().c_str(), LOG_LEVEL_INFO);
    NS_LOG_INFO("setting up ns3 simulation setup of " << exampleName());
    std::cout << "setting up ns3 simulation setup of " << exampleName() << " run count:" << runCount <<std::endl;

    uint16_t numOfPairs = 5;
    uint16_t numberOfEnbs = numOfPairs;
    uint16_t numberOfUes = numOfPairs;
    Time simTime = Seconds (30);
    double distance = 120.0;
    Time interPacketInterval = MilliSeconds (1000);
    uint64_t packetSize = 20; // in Bytes - ns3 max is 63KB
    bool useCa = false;
    bool disableDl = false;
    bool disableUl = false;
    bool disablePl = true;
    bool buildings = true;

    Config::SetDefault ("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue (false));
    Config::SetDefault ("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue (true));
    Config::SetDefault ("ns3::LteAmc::AmcModel", EnumValue (LteAmc::PiroEW2010));
    Config::SetDefault ("ns3::LteAmc::Ber", DoubleValue (0.00005));

    RngSeedManager::SetSeed (1234);
    RngSeedManager::SetRun (runCount);

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


    NodeContainer ueNodes;
    NodeContainer enbNodes;
    enbNodes.Create(numberOfEnbs);
    ueNodes.Create(numberOfUes);

    std::vector<VehicleProperties> vehiclesProps = generateCarProps(numberOfUes);
    std::vector<VehicleProperties> UAVProps = generateUAVProps(numberOfEnbs);
    vehiclesProps.insert(vehiclesProps.begin(), UAVProps.begin(), UAVProps.end());


    MobilityHelper mobility;
    NodeContainer vehicles (enbNodes, ueNodes);
    setupNS3MobilityFromVehicleProps(mobility, vehiclesProps, vehicles);

    BuildingsHelper::Install (enbNodes);
    BuildingsHelper::Install (ueNodes);

    // Configure lte antennas
//    lteHelper->SetEnbAntennaModelType("ns3::ParabolicAntennaModel");
//    lteHelper->SetEnbAntennaModelAttribute("Beamwidth", DoubleValue(60));
//    lteHelper->SetEnbAntennaModelAttribute("Orientation", DoubleValue(150));

    // Install LTE Devices to the nodes
    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);

    // Assign Ip Address to the EnB(UAV)
    for (uint32_t u = 0; u < enbNodes.GetN(); u++){
        Ptr<Ipv4> ipv4 = enbNodes.Get(u)->GetObject<Ipv4>();
        int32_t interface = ipv4->GetInterfaceForDevice (enbLteDevs.Get(u));
        if (interface == -1) {
            interface = static_cast<int32_t>(ipv4->AddInterface (enbLteDevs.Get(u)));
        }
        uint32_t interface_casted = static_cast<uint32_t>(interface);
        std::string ipaddress = "7.0.0." + std::to_string(u+1);
        Ipv4InterfaceAddress ipv4Addr = Ipv4InterfaceAddress (Ipv4Address(ipaddress.c_str()), Ipv4Mask ("/16"));
        ipv4->AddAddress (interface_casted, ipv4Addr);
        ipv4->SetMetric (interface_casted, 1);
        ipv4->SetUp (interface_casted);
    }

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
    for (uint16_t i = 0; i < numOfPairs; i++)
    {
        lteHelper->Attach (ueLteDevs.Get(i), enbLteDevs.Get(i));
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


    // Attach MobilityData Applications on each  ENB & UE pair
    for (uint32_t n = 0; n < enbNodes.GetN(); n++) {
        Ptr<Node> node = enbNodes.Get (n); // Get pointer to ith node in container
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> (); // Get Ipv4 instance of the node
        Ipv4Address addr = ipv4->GetAddress (3, 0).GetLocal (); // Get Ipv4InterfaceAddress of xth interface.
        std::cout << addr << std::endl;
        Config::Set ("/NodeList/"+ std::to_string(ueNodes.Get(n)->GetId()) +"/ApplicationList/*/$MobilitySenderUDP/Destination",
                     Ipv4AddressValue (addr));
    }

    for (uint i = 0; i < enbLteDevs.GetN(); i++) {
        Ptr<LteEnbNetDevice> lteEnbDev = enbLteDevs.Get (i)->GetObject<LteEnbNetDevice> ();
        Ptr<LteEnbPhy> enbPhy = lteEnbDev->GetPhy ();

        // Set transmission power and noise figure
        enbPhy->SetAttribute ("TxPower", DoubleValue (43.0));
        enbPhy->SetAttribute ("NoiseFigure", DoubleValue (5.0));
    }


    // Set UE and power
    for (uint i = 0; i < ueLteDevs.GetN(); i++)
    {
        Ptr<LteUeNetDevice> lteUeDev = ueLteDevs.Get (i)->GetObject<LteUeNetDevice> ();
        Ptr<LteUePhy> uePhy = lteUeDev->GetPhy ();
        uePhy->SetAttribute ("TxPower", DoubleValue (23.0));
        uePhy->SetAttribute ("NoiseFigure", DoubleValue (9.0));
    }

    LogComponentEnable("ParabolicAntennaModel", LOG_LEVEL_LOGIC);

    serverApps.Start (MilliSeconds (10));
    clientApps.Start (MilliSeconds (40));
    mobilitySenderApps.Start (MilliSeconds (40));
    mobilityReceiverApps.Start (MilliSeconds (10));
    lteHelper->EnablePdcpTraces();

    Ptr<RadioBearerStatsCalculator> pdcpStats = lteHelper->GetPdcpStats ();
    pdcpStats->SetDlPdcpOutputFilename("../results/DlPdcpStats_" + exampleName() + "_" + std::to_string(runCount) + ".txt");
    pdcpStats->SetUlPdcpOutputFilename("../results/UlPdcpStats_" + exampleName() + "_" + std::to_string(runCount) + ".txt");
    Config::SetDefault ("ns3::PhyStatsCalculator::DlRsrpSinrFilename", StringValue("../results/DlRsrpSinrStats_"+ exampleName() + "_" + std::to_string(runCount) + ".txt"));

    Simulator::Stop (simTime);
    wireUpLIMoSimAndStartSimulation();


}

} // namespace LteBeamSteering
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim
