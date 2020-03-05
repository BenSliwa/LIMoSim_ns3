#include "ltesimpleepcmobile.h"

#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store.h"
#include <ns3/propagation-module.h>
#include <ns3/buildings-helper.h>

// LIMoSim ns3
#include "ns3/ns3setuphelpers.h"
#include "ns3/limosimmobilitymodel.h"
//#include "ns3/applications/trafficanimation.h"

// LIMoSim
#include "LIMoSim/world/vehiclemanager.h"
//#include "LIMoSim/mobility/uav/reynolds/behaviors.h"


namespace LIMoSim {
namespace NS3 {
namespace Examples {

using namespace ns3;
using Node = ns3::Node;
//using namespace Applications;

NS_LOG_COMPONENT_DEFINE ("LteSimpleEpcMobile");

namespace LteSimpleEpcMobile {

Behavior* behaviorFactory() {
    return new SimpleBehavior();
}

void setup(int runCount)
{

    LogComponentEnable("LteSimpleEpcMobile", LOG_LEVEL_INFO);
    NS_LOG_INFO("setting up ns3 simulation setup of " << "LteSimpleEpcMobile");
    std::cout << "setting up ns3 simulation setup of " << "LteSimpleEpcMobile" << "run count:" << runCount <<std::endl;

    Config::SetDefault ("ns3::LteHelper::UseIdealRrc", BooleanValue (false));

    uint16_t numberOfEnbs = 1;
    uint16_t numberOfUes = 2;
    Time simTime = Seconds (5);
    double distance = 120.0;
    Time interPacketInterval = MilliSeconds (100);
    bool useCa = false;
    bool disableDl = false;
    bool disableUl = false;
    bool disablePl = false;


    if (useCa)
     {
       Config::SetDefault ("ns3::LteHelper::UseCa", BooleanValue (useCa));
       Config::SetDefault ("ns3::LteHelper::NumberOfComponentCarriers", UintegerValue (2));
       Config::SetDefault ("ns3::LteHelper::EnbComponentCarrierManager", StringValue ("ns3::RrComponentCarrierManager"));
     }


    Config::SetDefault ("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue (false));
    Config::SetDefault ("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue (true));
    Config::SetDefault ("ns3::LteAmc::AmcModel", EnumValue (LteAmc::PiroEW2010));
    Config::SetDefault ("ns3::LteAmc::Ber", DoubleValue (0.05));

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
//    lteHelper->SetAttribute ("PropagationDelayModel", StringValue ("ns3::ConstantSpeedPropagationDelayModel"));
    lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::FriisSpectrumPropagationLossModel"));
    lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::HybridBuildingsPropagationLossModel"));
    lteHelper->SetPathlossModelAttribute ("ShadowSigmaOutdoor", DoubleValue (0.0));
    lteHelper->SetPathlossModelAttribute ("ShadowSigmaIndoor", DoubleValue (0.0));
    lteHelper->SetPathlossModelAttribute ("ShadowSigmaExtWalls", DoubleValue (0.0));
    Config::SetDefault ("ns3::LteUePhy::EnableUplinkPowerControl", BooleanValue (false));

    lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler");
    lteHelper->SetSchedulerAttribute ("UlCqiFilter", EnumValue (FfMacScheduler::PUSCH_UL_CQI));


    Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
    lteHelper->SetEpcHelper (epcHelper);



//    ConfigStore inputConfig;
//    inputConfig.ConfigureDefaults();

    Ptr<Node> pgw = epcHelper->GetPgwNode ();

     // Create a single RemoteHost
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create (1);
    Ptr<Node> remoteHost = remoteHostContainer.Get (0);
    InternetStackHelper internet;
    internet.Install (remoteHostContainer);

    // Create the Internet
    AsciiTraceHelper ascii;
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
    p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
    p2ph.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (100)));
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

    // Setup LIMoSim Mobility
    std::vector<VehicleProperties> vehiclesProps = {
        VehicleProperties(
            Vector (1300, 300, 150),
            LIMoSimUAV,
            "U1",
            VehicleUIProps("yellow"),
            MakeCallback(&behaviorFactory)
                ),
        VehicleProperties(
            Vector (300, 0, 150),
            LIMoSimUAV,
            "U2",
            VehicleUIProps("green"),
            MakeCallback(&behaviorFactory)
                )
    };
    MobilityHelper mobility;
    setupNS3MobilityFromVehicleProps(mobility, vehiclesProps, ueNodes);

    // Install Mobility Model
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
    for (uint16_t i = 0; i < numberOfEnbs; i++)
    {
        positionAlloc->Add (Vector(distance * i, 0, 30)); // ENBs positions
    }
//    for (uint16_t i = 0; i < numberOfNodes; i++)
//    {
//        positionAlloc->Add (Vector(distance * i, distance,0)); // UEs positions
//    }
    positionAlloc->Add(Vector(distance/2, -distance/2, distance)); //  PGW position
    positionAlloc->Add(Vector(distance/2, -distance, 0)); // RH position

    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(enbNodes);
//    mobility.Install(ueNodes);
//    mobility.Install(pgw);
//    mobility.Install(remoteHostContainer);

    BuildingsHelper::Install (enbNodes);
    BuildingsHelper::Install (ueNodes);


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

    // Attach one UE per eNodeB
//    for (uint16_t i = 0; i < numberOfNodes; i++)
//    {
//        lteHelper->Attach (ueLteDevs.Get(i), enbLteDevs.Get(i));
//        // side effect: the default EPS bearer will be activated
//    }

    // Attach all UEs to first eNodeB
    for (uint16_t i = 0; i < numberOfUes; i++)
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
        if (!disableDl)
        {
            PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
            serverApps.Add (dlPacketSinkHelper.Install (ueNodes.Get(u)));

            UdpClientHelper dlClient (ueIpIface.GetAddress (u), dlPort);
            dlClient.SetAttribute ("Interval", TimeValue (interPacketInterval));
            dlClient.SetAttribute ("MaxPackets", UintegerValue (1000000));
            clientApps.Add (dlClient.Install (remoteHost));
        }

        if (!disableUl)
        {
            ++ulPort;
            PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
            serverApps.Add (ulPacketSinkHelper.Install (remoteHost));

            UdpClientHelper ulClient (remoteHostAddr, ulPort);
            ulClient.SetAttribute ("Interval", TimeValue (interPacketInterval));
            ulClient.SetAttribute ("MaxPackets", UintegerValue (1000000));
            clientApps.Add (ulClient.Install (ueNodes.Get(u)));
        }

        if (!disablePl && numberOfUes > 1)
        {
            ++otherPort;
            PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), otherPort));
            serverApps.Add (packetSinkHelper.Install (ueNodes.Get(u)));

            UdpClientHelper client (ueIpIface.GetAddress (u), otherPort);
            client.SetAttribute ("Interval", TimeValue (interPacketInterval));
            client.SetAttribute ("MaxPackets", UintegerValue (1000000));
            clientApps.Add (client.Install (ueNodes.Get ((u + 1) % numberOfUes)));
        }
    }


    Ptr<LteEnbNetDevice> lteEnbDev = enbLteDevs.Get (0)->GetObject<LteEnbNetDevice> ();
    Ptr<LteEnbPhy> enbPhy = lteEnbDev->GetPhy ();
    enbPhy->SetAttribute ("TxPower", DoubleValue (43.0));
    enbPhy->SetAttribute ("NoiseFigure", DoubleValue (5.0));

    // Set UEs' position and power
    for (int i = 0; i < ueLteDevs.GetN(); i++)
    {
        Ptr<LteUeNetDevice> lteUeDev = ueLteDevs.Get (i)->GetObject<LteUeNetDevice> ();
        Ptr<LteUePhy> uePhy = lteUeDev->GetPhy ();
        uePhy->SetAttribute ("TxPower", DoubleValue (23.0));
        uePhy->SetAttribute ("NoiseFigure", DoubleValue (9.0));
    }

    // Activate an EPS bearer
//    enum EpsBearer::Qci q = EpsBearer::GBR_CONV_VOICE;
//    EpsBearer bearer (q);
//    lteHelper->ActivateDataRadioBearer (ueLteDevs, bearer);

    serverApps.Start (MilliSeconds (10));
    clientApps.Start (MilliSeconds (40));
    lteHelper->EnableTraces ();

//    Ptr<RadioBearerStatsCalculator> pdcpStats = lteHelper->GetPdcpStats ();
//    pdcpStats->SetDlPdcpOutputFilename("DlPdcpStats_" + std::to_string(runCount) + ".txt");

    // Enable tracing
//    internet.EnableAsciiIpv4All(ascii.CreateFileStream ("../tracing/lte-simple-epc-static-internet.tr"));
//    internet.EnablePcapIpv4All("../tracing/lte-simple-epc-static-internet");
//    p2ph.EnableAsciiAll (ascii.CreateFileStream ("../tracing/lte-simple-epc-static-p2p.tr"));
//    p2ph.EnablePcapAll ("../tracing/lte-simple-epc-static-p2p");
    //lteHelper->EnableLogComponents();


//    setupIpv4TrafficAnimation();
//    setupLtePdcpTrafficAnimation();
//    setupLteUeMeasurementsCallbacks();

    Simulator::Stop (simTime);
    wireUpLIMoSimAndStartSimulation();
}



void callbackLteReportUeMeasurement(std::string path, uint16_t rnti, uint16_t cellId, double rsrp, double rsrq, bool servingCell, uint8_t componentCarrierId)
{
    std::cout << "Ue " << extractNodeId(path) <<": RSRP " << rsrp << std::endl;
}



SimpleBehavior::SimpleBehavior(std::string _agentId):
    Behavior("LteSimpleEpcCustomBehavior", _agentId),
    m_agentId(_agentId),
    m_trip(true),
    m_behavior_trip(Behavior_Seek(Vector3d(10,10,60),_agentId)),
    m_behavior_return(Behavior_Seek(Vector3d(200,200,150),_agentId))
{

}

Steering SimpleBehavior::apply()
{
    // Direction control
    if ((getAgent()->getPosition() - Vector3d(10,10,60)).norm() < 10 && m_trip) {
        m_trip = !m_trip;
    }

    if (m_trip) {
        return m_behavior_trip.apply();
    } else {
        return m_behavior_return.apply();
    }

}

void SimpleBehavior::setAgent(std::string _agentId)
{
    Behavior::setAgent(_agentId);
    m_behavior_trip.setAgent(_agentId);
    m_behavior_return.setAgent(_agentId);
}

}

}
}
}
