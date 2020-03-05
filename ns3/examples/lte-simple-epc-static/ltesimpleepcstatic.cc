#include "ltesimpleepcstatic.h"

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


#include "ns3/ns3setuphelpers.h"

#include "ns3/applications/trafficanimation.h"

#include "LIMoSim/simulation/simulation.h"

namespace LIMoSim {
namespace NS3 {
namespace Examples {

using namespace ns3;
using namespace Applications;

NS_LOG_COMPONENT_DEFINE ("LteSimpleEpcStatic");

namespace LteSimpleEpcStatic {

void setup()
{

    LogComponentEnable("LteSimpleEpcStatic", LOG_LEVEL_INFO);
    NS_LOG_INFO("setting up ns3 simulation setup of " << "LteSimpleEpcStatic");
    std::cout << "setting up ns3 simulation setup of " << "LteSimpleEpcStatic" <<std::endl;

    uint16_t numberOfNodes = 2;
    Time simTime = Seconds (10);
    double distance = 60.0;
    Time interPacketInterval = MilliSeconds (100);
    bool useCa = false;
    bool disableDl = false;
    bool disableUl = false;
    bool disablePl = false;

    // Command line arguments
//    CommandLine cmd;
//    cmd.AddValue ("numberOfNodes", "Number of eNodeBs + UE pairs", numberOfNodes);
//    cmd.AddValue ("simTime", "Total duration of the simulation", simTime);
//    cmd.AddValue ("distance", "Distance between eNBs [m]", distance);
//    cmd.AddValue ("interPacketInterval", "Inter packet interval", interPacketInterval);
//    cmd.AddValue ("useCa", "Whether to use carrier aggregation.", useCa);
//    cmd.AddValue ("disableDl", "Disable downlink data flows", disableDl);
//    cmd.AddValue ("disableUl", "Disable uplink data flows", disableUl);
//    cmd.AddValue ("disablePl", "Disable data flows between peer UEs", disablePl);
//    cmd.Parse(argc, argv);

    if (useCa)
     {
       Config::SetDefault ("ns3::LteHelper::UseCa", BooleanValue (useCa));
       Config::SetDefault ("ns3::LteHelper::NumberOfComponentCarriers", UintegerValue (2));
       Config::SetDefault ("ns3::LteHelper::EnbComponentCarrierManager", StringValue ("ns3::RrComponentCarrierManager"));
     }

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
    Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
    lteHelper->SetEpcHelper (epcHelper);

//    ConfigStore inputConfig;
//    inputConfig.ConfigureDefaults();

    // parse again so you can override default values from the command line
    //cmd.Parse(argc, argv);

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
    enbNodes.Create(numberOfNodes);
    ueNodes.Create(numberOfNodes);

    // Install Mobility Model
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
    for (uint16_t i = 0; i < numberOfNodes; i++)
    {
        positionAlloc->Add (Vector(distance * i, 0, distance)); // ENBs positions
    }
    for (uint16_t i = 0; i < numberOfNodes; i++)
    {
        positionAlloc->Add (Vector(distance * i, distance,0)); // UEs positions
    }
    positionAlloc->Add(Vector(distance/2, -distance/2, distance)); //  PGW position
    positionAlloc->Add(Vector(distance/2, -distance, 0)); // RH position
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(enbNodes);
    mobility.Install(ueNodes);
//    mobility.Install(pgw);
//    mobility.Install(remoteHostContainer);

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
    for (uint16_t i = 0; i < numberOfNodes; i++)
      {
        lteHelper->Attach (ueLteDevs.Get(i), enbLteDevs.Get(i));
        // side effect: the default EPS bearer will be activated
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

        if (!disablePl && numberOfNodes > 1)
        {
            ++otherPort;
            PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), otherPort));
            serverApps.Add (packetSinkHelper.Install (ueNodes.Get(u)));

            UdpClientHelper client (ueIpIface.GetAddress (u), otherPort);
            client.SetAttribute ("Interval", TimeValue (interPacketInterval));
            client.SetAttribute ("MaxPackets", UintegerValue (1000000));
            clientApps.Add (client.Install (ueNodes.Get ((u + 1) % numberOfNodes)));
        }
    }

    // Enable tracing
//    internet.EnableAsciiIpv4All(ascii.CreateFileStream ("../tracing/lte-simple-epc-static-internet.tr"));
//    internet.EnablePcapIpv4All("../tracing/lte-simple-epc-static-internet");
//    p2ph.EnableAsciiAll (ascii.CreateFileStream ("../tracing/lte-simple-epc-static-p2p.tr"));
//    p2ph.EnablePcapAll ("../tracing/lte-simple-epc-static-p2p");

//    Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
//      MakeCallback (&NotifyConnectionEstablishedEnb));
//    Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
//     MakeCallback (&NotifyConnectionEstablishedUe));

//    Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/RandomAccessSuccessful",
//             MakeCallback (&NotifyRandomAccessSuccessfulUe));
// "/NodeList/2/DeviceList/0/LteEnbRrc/UeMap/1/Srb0/LteRlc/TxPDU"
// "/NodeList/4/DeviceList/0/LteUeRrc"
//    Simulator::Schedule(
//                MilliSeconds(25),
//                [&]() {
//                    using namespace TraceCallbacks;
//                    Config::Connect ("/NodeList/*/DeviceList/*/LteUePhy/ReportUeMeasurements",
//                      MakeCallback (&callbackLteReportUeMeasurement));
//                    Config::Connect ("/NodeList/4/DeviceList/*/LteUePhy/ReportCurrentCellRsrpSinr",
//                      MakeCallback (&callbackLteReportCellRsrpSnriMeasurement));
//                    Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/UeMap/*/Srb0/LteRlc/TxPDU",
//                      MakeCallback (&callBackLtePdcpUeTxPdu));
////                    Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/DataRadioBearerMap/*/LteRlc/RxPDU",
////                                       MakeCallback (&Lte_RlcDlRxPDU));
//                }
//                );

//    Config::Connect ("/NodeList/*/DeviceList/*/LteUePhy/ReportUeMeasurements",
//      MakeCallback (&callbackLteReportUeMeasurement));

//    setupIpv4TrafficAnimation();
    //setupLtePdcpTrafficAnimation();
    lteHelper->EnableRlcTraces();
//    lteHelper->EnableTraces ();
    // setupLteUeMeasurementsCallbacks();

    serverApps.Start (MilliSeconds (10));
    clientApps.Start (MilliSeconds (40));

    Simulator::Stop (simTime);

    wireUpLIMoSimAndStartSimulation();
}

void NotifyRandomAccessSuccessfulUe(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti) {
    ConnectSrb0Traces(context, imsi, cellId, rnti);
}

void ConnectSrb0Traces(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti) {
    std::string ueRrcPath =  context.substr (0, context.rfind ("/"));

    using namespace TraceCallbacks;
    Config::Connect (ueRrcPath + "/Srb0/LteRlc/TxPDU",
                     MakeCallback (&callBackLtePdcpUeTxPdu));
}

void NotifyConnectionEstablishedUe(std::string context,
                                   uint64_t imsi,
                                   uint16_t cellid,
                                   uint16_t rnti)
{
    std::cout << context
                 << " UE IMSI " << imsi
                 << ": connected to CellId " << cellid
                 << " with RNTI " << rnti
      << std::endl;
    ConnectSrb0Traces(context, imsi, cellid, rnti);
    using namespace TraceCallbacks;
    Config::Connect ("/NodeList/*/DeviceList/*/LteUePhy/ReportUeMeasurements",
      MakeCallback (&callbackLteReportUeMeasurement));
}

void NotifyConnectionEstablishedEnb (std::string context,
                                uint64_t imsi,
                                uint16_t cellid,
                                uint16_t rnti)
{
  std::cout << context
            << " eNB CellId " << cellid
            << ": successful connection of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
}

void Lte_RlcDlRxPDU (std::string context, uint16_t rnti, uint8_t lcid, uint32_t packetSize, uint64_t delay)
{
  std::cout << "RLC: Received DL Rx PDU at rnti  " << rnti
        << " Packet size in Bytes " << packetSize << " Delay  " << delay << std::endl;
}

}

}
}
}
