#include "lteaerialbasestationcluster.h"

// ns3
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
#include "LIMoSim/simulation/performancemonitor.h"
#include "LIMoSim/world/vehiclemanager.h"


#include "ui/data/uidatamanager.h"

namespace LIMoSim {
namespace NS3 {
namespace Examples {
namespace LteAerialBasestationCluster {

using namespace ns3;
using Node = ns3::Node;

inline std::string exampleName() {
    return "LteAerialBasestationCluster";
}

NS_LOG_COMPONENT_DEFINE (exampleName());

Behavior* behaviorFactory(Vector3d _regionTopLeft, double _regionHeight, double _regionWidth) {
    return new Behavior_Cohesion2DRegionwise(_regionTopLeft, _regionHeight, _regionWidth);
}


std::vector<VehicleProperties> generateCarProps(uint numberOfCars) {
    std::vector<VehicleProperties> vehiclesProps;
    for (uint i = 0; i < numberOfCars; i++) {
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

static std::map<uint64_t, uint16_t> imsiToCellId;

NetDeviceContainer* ueLteDevsPointer(NetDeviceContainer * _netdevs = nullptr) {
    static NetDeviceContainer * pointer = _netdevs;
    return pointer;
}

NetDeviceContainer* enbLteDevsPointer(NetDeviceContainer *_netdevs = nullptr) {
    static NetDeviceContainer * pointer = _netdevs;
    return pointer;
}

std::string nodeIdFromImsi(uint64_t _imsi, NetDeviceContainer& _ueLteDevs) {
    for (uint16_t i = 0; i < _ueLteDevs.GetN(); i++) {
        Ptr<LteUeNetDevice> netDev = DynamicCast<LteUeNetDevice>(_ueLteDevs.Get(i));
        uint64_t imsi= netDev->GetImsi();
        if (_imsi == netDev->GetImsi()) {
            return std::to_string(netDev->GetNode()->GetId());
        }
    }
    return "";
}

std::string nodeIdFromCellId(uint64_t _cellId, NetDeviceContainer& _enbLteDevs) {
    for (uint16_t i = 0; i < _enbLteDevs.GetN(); i++) {
        Ptr<LteEnbNetDevice> netDev = DynamicCast<LteEnbNetDevice>(_enbLteDevs.Get(i));
        uint64_t cellId= netDev->GetCellId();
        if (_cellId == cellId) {
            return std::to_string(netDev->GetNode()->GetId());
        }
    }
    return "";
}

std::string antagonsitNodeIdFromCellId(uint64_t _cellId, NetDeviceContainer& _enbLteDevs) {
    for (uint16_t i = 0; i < _enbLteDevs.GetN(); i++) {
        Ptr<LteEnbNetDevice> netDev = DynamicCast<LteEnbNetDevice>(_enbLteDevs.Get(i));
        uint64_t cellId= netDev->GetCellId();
        if (_cellId != cellId) {
            return std::to_string(netDev->GetNode()->GetId());
        }
    }
    return "";
}


void
NotifyConnectionEstablishedUe (std::string context,
                               uint64_t imsi,
                               uint16_t cellid,
                               uint16_t rnti)
{
//  std::cout << Simulator::Now ().GetSeconds () << " " << context
//            << " UE IMSI " << imsi
//            << ": connected to CellId " << cellid
//            << " with RNTI " << rnti
//            << std::endl;
//  imsiToCellId[imsi] = cellid;
//  auto uidm = UI::Data::UIDataManager::getInstance();
//  std::string node1 = std::to_string(cellid + 4);
//  std::string _node1 = std::to_string((!(cellid + 4)%5) + 5);
//  std::string node2 = nodeIdFromImsi(imsi, *ueLteDevsPointer());
//  if (!node2.empty()) {
//      uidm->unregisterConnection(_node1, node2);
//      uidm->registerConnection(node1, node2);
//  }
}

void
NotifyHandoverStartUe (std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti,
                       uint16_t targetCellId)
{
//  std::cout << Simulator::Now ().GetSeconds () << " " << context
//            << " UE IMSI " << imsi
//            << ": previously connected to CellId " << cellid
//            << " with RNTI " << rnti
//            << ", doing handover to CellId " << targetCellId
//            << std::endl;
}

void
NotifyHandoverEndOkUe (std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti)
{
  std::cout << Simulator::Now ().GetSeconds () << " " << context
            << " UE IMSI " << imsi
            << ": successful handover to CellId " << cellid
            << " with RNTI " << rnti
            << std::endl;
  imsiToCellId[imsi] = cellid;
  auto uidm = UI::Data::UIDataManager::getInstance();
  std::string node1 = nodeIdFromCellId(cellid, *enbLteDevsPointer());
  std::string _node1 = antagonsitNodeIdFromCellId(cellid, *enbLteDevsPointer());
  std::string node2 = nodeIdFromImsi(imsi, *ueLteDevsPointer());
  if (!node2.empty()) {
      uidm->unregisterConnection(_node1, node2);
      uidm->registerConnection(node1, node2);
  }
}

void
NotifyConnectionEstablishedEnb (std::string context,
                                uint64_t imsi,
                                uint16_t cellid,
                                uint16_t rnti)
{
  std::cout << Simulator::Now ().GetSeconds () << " " << context
            << " eNB CellId " << cellid
            << ": successful connection of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
    imsiToCellId[imsi] = cellid;
    auto uidm = UI::Data::UIDataManager::getInstance();
    std::string node1 = nodeIdFromCellId(cellid, *enbLteDevsPointer());
    std::string _node1 = antagonsitNodeIdFromCellId(cellid, *enbLteDevsPointer());
    std::string node2 = nodeIdFromImsi(imsi, *ueLteDevsPointer());
    if (!node2.empty()) {
        uidm->unregisterConnection(_node1, node2);
        uidm->registerConnection(node1, node2);
    }
}

void
NotifyHandoverStartEnb (std::string context,
                        uint64_t imsi,
                        uint16_t cellid,
                        uint16_t rnti,
                        uint16_t targetCellId)
{
//  std::cout << Simulator::Now ().GetSeconds () << " " << context
//            << " eNB CellId " << cellid
//            << ": start handover of UE with IMSI " << imsi
//            << " RNTI " << rnti
//            << " to CellId " << targetCellId
//            << std::endl;
}

void
NotifyHandoverEndOkEnb (std::string context,
                        uint64_t imsi,
                        uint16_t cellid,
                        uint16_t rnti)
{
//  std::cout << Simulator::Now ().GetSeconds () << " " << context
//            << " eNB CellId " << cellid
//            << ": completed handover of UE with IMSI " << imsi
//            << " RNTI " << rnti
//            << std::endl;
}

void setupUIConnections(NetDeviceContainer _ueLteDevs) {
    auto uidm = UI::Data::UIDataManager::getInstance();
//    uidm->clearConnections();
    for (uint16_t i = 0; i < _ueLteDevs.GetN(); i++) {
        Ptr<LteUeNetDevice> netDev = DynamicCast<LteUeNetDevice>(_ueLteDevs.Get(i));
        uint16_t cellId = imsiToCellId.at(netDev->GetImsi());
        std::string node1 = cellId == 1 ? "0" : "1";
        uidm->registerConnection(node1, std::to_string(netDev->GetNode()->GetId()));
    }
}

void updateEnbAssociation(Ptr<LteHelper> _lteHelper, NetDeviceContainer _ueLteDevs, NetDeviceContainer _enbLteDevs) {
//    std::cout << Simulator::Now().GetSeconds() <<  ": checking for updates in enb association" << std::endl;
    auto uidm = UI::Data::UIDataManager::getInstance();
    for (uint16_t i = 0; i < _ueLteDevs.GetN(); i++) {
        Ptr<ns3::MobilityModel> mobility = _ueLteDevs.Get(i)->GetNode()->GetObject<ns3::MobilityModel>();
        Ptr<LteUeNetDevice> netDev = DynamicCast<LteUeNetDevice>(_ueLteDevs.Get(i));
        Vector pos = mobility->GetPosition();
        uint16_t cellId = imsiToCellId.at(netDev->GetImsi());
        if (pos.x <= 500 && cellId == 2) {
            std::cout <<"enb1 ID: " << std::to_string(_enbLteDevs.Get (0)->GetNode()->GetId()) << std::endl;
            _lteHelper->HandoverRequest (MilliSeconds (250), _ueLteDevs.Get (i), _enbLteDevs.Get (1), _enbLteDevs.Get (0));
//            uidm->unregisterConnection("6", std::to_string(netDev->GetNode()->GetId()));
//            uidm->registerConnection("5", std::to_string(netDev->GetNode()->GetId()));
        } else if(pos.x > 500 && cellId == 1) {
            std::cout <<"enb2 ID " << std::to_string(_enbLteDevs.Get (0)->GetNode()->GetId()) << std::endl;
            _lteHelper->HandoverRequest (MilliSeconds (250), _ueLteDevs.Get (i), _enbLteDevs.Get (0), _enbLteDevs.Get (1));
//            uidm->unregisterConnection("5", std::to_string(netDev->GetNode()->GetId()));
//            uidm->registerConnection("6", std::to_string(netDev->GetNode()->GetId()));
        }
    }

    // Schedule next check
    Simulator::Schedule(Seconds(1), &updateEnbAssociation, _lteHelper, _ueLteDevs, _enbLteDevs);
}


void setup(int runCount, uint16_t numOfUAVs, uint16_t numOfCars, uint16_t simTime_s, uint16_t interPacketInterval_ms, uint16_t packetSize_B)
{
    LogComponentEnable(exampleName().c_str(), LOG_LEVEL_INFO);
    NS_LOG_INFO("setting up ns3 simulation setup of " << exampleName());
    std::cout << "setting up ns3 simulation setup of " << exampleName() << " run count:" << runCount <<std::endl;
    Simulation::getInstance()->setName(exampleName() + "_" +
                                       "t" + std::to_string(simTime_s)+ "_" +
                                       "nu" + std::to_string(numOfUAVs) + "_"+
                                       "nc" + std::to_string(numOfCars) + "_"+
                                       "ipi"+ std::to_string(interPacketInterval_ms) + "_" +
                                       "ps" + std::to_string(packetSize_B))->setRunCount(runCount);
    std::cout << "Scenario params: " <<std::endl;
    std::cout << "- sim time: " << simTime_s << " seconds" << std::endl;
    std::cout << "- num of UAVs: " << numOfUAVs << std::endl;
    std::cout << "- num of Cars: " << numOfCars << std::endl;
    std::cout << "- inter packet interval: " << interPacketInterval_ms << " seconds"<< std::endl;
    std::cout << "- packet size: " << packetSize_B << " Bytes" << std::endl;

    PerformanceMonitor::getInstance()->reset();
    PerformanceMonitor::getInstance()->setupWithExport("../results/Performance_" +
                                                       Simulation::getInstance()->getName() + "_" +
                                                       std::to_string(runCount) +
                                                       ".csv");

    uint16_t numOfEnbs = numOfUAVs;
    uint16_t numOfUes = numOfCars;
    Time simTime = Seconds (simTime_s);
    Time interPacketInterval = MilliSeconds(interPacketInterval_ms);
    uint64_t packetSize = packetSize_B; // in Bytes - ns3 max is 63KB

    bool disableDl = false;
    bool disableUl = false;
    bool disablePl = true;
    bool showProgress = true;
    bool buildings = true;

    uint32_t dlEarfcn = 100;
    uint32_t ulEarfcn = 18100;
    uint32_t dlResBlck = 100;
    uint32_t ulResBlck = 100;

    double eNBTxPower = 43.0;
    double ueTxPower = 23.0;

    imsiToCellId.clear();

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults ();

    srand(static_cast<unsigned int>(time(nullptr)));
    uint seed = static_cast<uint>(rand());
    RngSeedManager::SetSeed (seed);
    RngSeedManager::SetRun (static_cast<uint>(runCount));

    Config::SetDefault ("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue (false));
    Config::SetDefault ("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue (false));
    Config::SetDefault ("ns3::LteUePhy::EnableUplinkPowerControl", BooleanValue (false));
    Config::SetDefault ("ns3::LteAmc::AmcModel", EnumValue (LteAmc::PiroEW2010));
    Config::SetDefault ("ns3::LteAmc::Ber", DoubleValue (0.00005));
    Config::SetDefault ("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue(ulResBlck));
    Config::SetDefault ("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue(dlResBlck));
    Config::SetDefault ("ns3::LteEnbNetDevice::UlEarfcn", UintegerValue(ulEarfcn));
    Config::SetDefault ("ns3::LteEnbNetDevice::DlEarfcn", UintegerValue(dlEarfcn));
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


    lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler");
    lteHelper->SetSchedulerAttribute ("UlCqiFilter", EnumValue (FfMacScheduler::PUSCH_UL_CQI));
    lteHelper->SetHandoverAlgorithmType ("ns3::NoOpHandoverAlgorithm");

    Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
    lteHelper->SetEpcHelper (epcHelper);
    // lteHelper->SetHandoverAlgorithmType ("ns3::A3RsrpHandoverAlgorithm");
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
    NodeContainer ueNodes;
    ueNodes.Create(numOfUes);
    enbNodes.Create(numOfEnbs);


    // Setup Mobility
    std::vector<VehicleProperties> carProps = generateCarProps(numOfUes);
    std::vector<VehicleProperties> uavProps {
        VehicleProperties(Vector(200, 300, 50),
                          LIMoSimUAV,
                          "U0",
                          VehicleUIProps("yellow"),
                          MakeBoundCallback(&behaviorFactory, Vector3d(-100, 3000), 8000.0, 500.0)),
        VehicleProperties(Vector(700, 300, 50),
                          LIMoSimUAV,
                          "U1",
                          VehicleUIProps("cyan"),
                          MakeBoundCallback(&behaviorFactory, Vector3d(501, 3000), 8000.0, 900.0)),

                                            };

    MobilityHelper mobilityHelper;
    setupNS3MobilityFromVehicleProps(mobilityHelper, uavProps, enbNodes);
    setupNS3MobilityFromVehicleProps(mobilityHelper, carProps, ueNodes);


    if (buildings){
        BuildingsHelper::Install (enbNodes);
        BuildingsHelper::Install (ueNodes);
        BuildingsHelper::MakeMobilityModelConsistent();
    }

    createLIMoSimVehicles();
    registerStaticNodes();

    // Install LTE Devices to the nodes
    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);

    enbLteDevsPointer(&enbLteDevs);
    ueLteDevsPointer(&ueLteDevs);


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


    // Assign the UEs to the cluster eNodeB
    lteHelper->AttachToClosestEnb(ueLteDevs, enbLteDevs);


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
                client.SetAttribute ("PacketSize", UintegerValue(packetSize));
                clientApps.Add (client.Install (ueNodes.Get (v)));
            }
        }
    }

    // Add X2 interface
    lteHelper->AddX2Interface (enbNodes);

    // randomize a bit start times to avoid simulation artifacts
    // (e.g., buffer overflows due to packet transmissions happening
    // exactly at the same time)
    Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
    startTimeSeconds->SetAttribute ("Min", DoubleValue (0.01));
    startTimeSeconds->SetAttribute ("Max", DoubleValue (0.06));

    // Display simulator time progress
    if (showProgress)
    {
        uint32_t status_period = 1;
        Simulator::Schedule(Seconds(1),&Callbacks::PrintProgress, status_period);
    }

    // Start Apps
    serverApps.Start (Seconds(startTimeSeconds->GetValue()));
    clientApps.Start (Seconds(startTimeSeconds->GetValue() + 0.04));
//    serverApps.Start (MilliSeconds (10));
//    clientApps.Start (MilliSeconds (40));

    Simulator::Schedule(Seconds(startTimeSeconds->GetValue() + 1), &updateEnbAssociation, lteHelper, ueLteDevs, enbLteDevs);


    lteHelper->EnablePdcpTraces();
//    lteHelper->EnablePhyTraces();

    Ptr<RadioBearerStatsCalculator> pdcpStats = lteHelper->GetPdcpStats ();
    pdcpStats->SetDlPdcpOutputFilename("../results/DlPdcpStats_" + Simulation::getInstance()->getName() + "_" + std::to_string(runCount) + ".txt");
    pdcpStats->SetUlPdcpOutputFilename("../results/UlPdcpStats_" + Simulation::getInstance()->getName() + "_" + std::to_string(runCount) + ".txt");

//    Config::SetDefault ("ns3::PhyStatsCalculator::DlRsrpSinrFilename", StringValue("../results/DlRsrpSinrStats_"+ Simulation::getInstance()->getName() + "_" + std::to_string(runCount) + ".txt"));


    // connect custom trace sinks for RRC connection establishment and handover notification
   Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                    MakeCallback (&NotifyConnectionEstablishedEnb));
   Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
                    MakeCallback (&NotifyConnectionEstablishedUe));
//   Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
//                    MakeCallback (&NotifyHandoverStartEnb));
//   Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
//                    MakeCallback (&NotifyHandoverStartUe));
//   Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
//                    MakeCallback (&NotifyHandoverEndOkEnb));
   Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
      MakeCallback (&NotifyHandoverEndOkUe));

    VehicleManager::getInstance()->enableMobilityBroadcastHelper();

    Simulator::Stop (simTime);
//    wireUpLIMoSimAndStartSimulation();
    startSimulation();
}

} // namespace LteAerialBasestationCluster
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim
